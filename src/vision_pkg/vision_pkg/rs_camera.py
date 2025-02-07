""" 
    RS_CAMERA:
    
    Description:


"""

# Import Dependencies
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import numpy as np
import rclpy
import pyrealsense2 as rs
import cv2 as cv

# Working on April Tag
import apriltag 

#########
# Camera Node
class CameraNode( Node ):

    # Initiallize Node
    def __init__(self, poll_period_sec: float=0.250) -> None:
        
        # Initiallize Camera Node 
        super().__init__("camera_node")
        self.get_logger().info("[ Camera.INIT ] Initiallized")

        # Initiallize Camera
        self.init_camera()

        # Initiallize Publisher
        self.init_comp_publisher(
            rgb_topic_name="rs_node/compressed_video",
            depth_topic_name="rs_node/depth_video"
        )

        # Initiallize AprilTag Detector
        self.init_detector()

        # Initialize Timer
        self.timer_counter = 0
        self.init_timer(period=poll_period_sec)
        return

    # Destructor
    def __del__(self) -> None:
        self.get_logger().info("[ Camera.DEL ] Destroying Camera Node")
        if self.pipeline: self.pipeline.stop()

        return

    ### CameraNode: CALLBACK FUNCTIONS  ###

    # Timer Callback Function
    def callback_timer(self) -> None:
        self.timer_counter += 1

        # Collect Frames + Error Handling
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().error(f"[ Camera.CALLBACK_TIMER ] Error: {e}")
            return
        
        # video_frame = frames.get_depth_frame()
        video_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # If Empty, Return
        if not video_frame: return
        if not depth_frame: return

        # Collect Data
        image = video_frame.as_frame().get_data()
        np_image = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2RGB)

        #####################################
        # TO-DO: Implement Depth Frame      #
        #####################################


        #####################################
        # TO-DO: IMU                        #
        #####################################
        # imu_frame = frames.get_motion_frame()

        # if imu_frame != None:
            # self.get_logger().warn(f"imu_frame collected")


        #####################################

        # Collect AprilTag Location
        np_grayscale = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2GRAY )
        result = self.detector.detect(np_grayscale)
        if len(result) == 0:
            self.get_logger().warn("[ CameraNode.timer ] No tag detected")
        else:
            self.get_logger().info(f"[ CameraNode.timer ]Tag ID: {result[0].tag_id}")

            # Collect Corner and Center Locations
            center = tuple(map(int, result[0].center))

            
            # Unpack Center Location
            center_x, center_y = center

            # Draw Corners and Center onto image 
            np_image = self.overlay_tag_corners(np_image, result[0])
            np_image = self.overlay_tag_center(np_image,  result[0])
            
            # Get Distance to Center
            # ISSUE: The grayscale image used to detect the april tag appears to have a different resolution
            #           than the depth frame.
            self.get_logger().info(f"[ DEBUG ] {center_x},{center_y}")
            if ( 0 <= center_x and center_x < depth_frame.get_width() ) and (0 <= center_y and center_y < depth_frame.get_height()):
                distance_center = depth_frame.get_distance( int(center_x), int(center_y) )
                cv.putText(np_image, f"{distance_center:0.3f}", (center_x - 20, center_y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 128, 40), 2)
                # self.get_logger().warn(f"Distance: {distance_center}")

        # Generate Message
        rgb_msg = CompressedImage()
        depth_msg = Image()

        # Populate RGB Message
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.format = "jpeg"

        # Populate Depth Message
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.encoding = "16UC1"

        # Pack Image Data
        rgb_msg.data = cv.imencode( ".jpg", np_image )[1].tobytes() # ChatGPT Suggested Fix

        # Send Messages
        self.pub_rgb.publish(msg=rgb_msg)
        self.pub_depth.publish(msg=depth_msg)

        # Frames Collected
        return

    ### CameraNode: Initiallizers       ###

    # Initiallize AprilTag Detector
    def init_detector(self) -> None:
        self.get_logger().info("[ CameraNode.init_detector ] Initiallizing Detector")
        self.detector = apriltag.Detector()
        self.get_logger().info("[ CameraNode.init_detector ] Detector Initiallized")
        return

    # Initiallize Timer
    def init_timer(self, period: float) -> None:
        self.get_logger().info(f"[ Camera.INIT_TIMER ] Starting Timer, Period={period}")
        
        self.timer = self.create_timer(
            timer_period_sec=period,
            callback=self.callback_timer
        )
        self.get_logger().info("[ Camera.INIT_TIMER ] Timer Initiallized")
        return

    # Initiallize Camera Pipeline
    def init_camera(self) -> None:
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting Camera")
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting IMU")

        # Initiallize Camera Pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Select, Configure, and Enable Streams in Pipeline configuration
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        # Send Configuration to Pipeline
        self.pipeline.start( config )
        self.get_logger().info("[ Camera.INIT_CAMERA ] Camera Initiallized")
        self.get_logger().info("[ Camera.INIT_CAMERA ] IMU Initiallized")
        return

    # Initiallize Compressed Image Publisher
    def init_comp_publisher(self, rgb_topic_name: str, depth_topic_name: str) -> None:
        self.get_logger().info("[ Camera.INIT_COMP_PUBLISHER ] Starting Compressed Image Publisher")
        self.pub_rgb = self.create_publisher(
            msg_type = CompressedImage,
            topic = rgb_topic_name,
            qos_profile = 3
        )
        self.get_logger().info(f"[ Camera.INIT_COMP_PUBLISHER ] RGB publisher On \033[31m/{rgb_topic_name}\033[0m Initiallized")
        
        self.pub_depth = self.create_publisher(
            msg_type = Image,
            topic = depth_topic_name,
            qos_profile = 3
        )
        self.get_logger().info(f"[ Camera.INIT_COMP_PUBLISHER ] Depth publisher On \033[31m/{rgb_topic_name}\033[0m Initiallized")
        return

    # Print Corners on AprilTag
    def overlay_tag_corners(self, np_image, tag):
        
        # Determine of the tag passed is REALLY a tag:
        #   If NONE, then passed tag does not exist 
        if tag == None or np_image.size == 0:
            return np.zeros(1)

        # Tag is real, what do we do now?
        else:
            # Collect Corners
            corners = tuple(map(list, tag.corners))

            # Print Corners to Image
            for i in range(4):
                cv.circle(np_image, ( int(corners[i][0]), int(corners[i][1]) ), 10, (255, 0, 0), 10)
            
            # Return Image (Find out if it is possible to modify by reference)
            return np_image

    # Print Center on AprilTag
    def overlay_tag_center(self, np_image, tag):
        # Determine of the tag passed is REALLY a tag:
        #   If NONE, then passed tag does not exist 
        if tag == None or np_image.size == 0:
            return np.zeros(1)

        # Tag is real, what do we do now?
        else:
            # Collect Center
            center_X, center_Y = tuple(map(int, tag.center))

            # Print Center to Image
            cv.circle(np_image, (center_X, center_Y), 5, (255, 0, 255), 5)

            # Return Image (Find out if it is possible to modify by reference)
            return np_image        



# Main Function
def main() -> int:
    rclpy.init()
    node = CameraNode(0.017)
    try:
        rclpy.spin(node=node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\n\033[100m[ Main.MAIN ] Keyboard Interrupt Pressed", end='\033[0m\n')

    return 0

if __name__ == "__main__":
    exit( main() )