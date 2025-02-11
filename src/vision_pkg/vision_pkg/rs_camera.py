""" 
    RS_CAMERA_Node:
    
        Description:
            Realsense Camera Node for handling a Single D455 Intel Realsense Camera.
            This node collects video and depth, as well as IMU acceleration and 
            gyroscope frames. It publishes to it's respective topics
            at a fixed frequency (i.e. 30FPS at 33 ms). A new frequency can be set
            by passing the desired period to the CameraNode.init_timer() method.
            


            Publishes four topics:
                Name                        Type                            Oneline-Description
            rs_node/compressed_video    sensor_msgs/msg/CompressedImage     Used to publish RGB uint8 video feed
            rs_node/depth_video         sensor_msgs/msg/Image               Used to publish depth camera footage
            rs_node/imu/accel_data      std_msgs/msg/Float32MultiArray      Used to publish x,y,z acceleration data
            rs_node/imu/gyro_data       std_msgs/msg/Float32MultiArray      Used to publish roll, pitch, yaw gyroscope data
"""

### Import Dependencies
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

########################################
# TODO: Replace w/ interface type geometry_msgs/msg/AccelStamped
# from example_interfaces.msg import Float32MultiArray
from geometry_msgs.msg import AccelStamped
########################################

import numpy as np
import rclpy
import pyrealsense2 as rs
import cv2 as cv
import apriltag 

###################
# Camera Node
class CameraNode( Node ):
    ##########
    # Initiallize Node
    def __init__(self, poll_period_sec: float=0.250) -> None:
        
        ## Initiallize Camera Node 
        super().__init__("camera_node")
        self.get_logger().info("[ Camera.INIT ] Initiallized")

        ## Initiallize Camera
        self.init_camera()

        ## Initiallize Publisher
        self.init_node_publisher(
            rgb_topic_name="rs_node/camera/compressed_video",      # Consider changing to rs_node/camera/rgb_video
            depth_topic_name="rs_node/camera/depth_video",         # Consider changing to rs_node/camera/depth_video
            imu_topic_name="rs_node/imu",
            imu_accel_topic_name="rs_node/imu/accel_info",
            imu_gyro_topic_name="rs_node/imu/gyro_info"
        )

        ## Initiallize AprilTag Detector
        self.init_detector()

        ## Initialize Timer
        self.timer_counter = 0
        self.start_time = self.get_clock().now()
        self.init_timer(period=poll_period_sec)
        return

    ##########
    # Destructor
    def __del__(self) -> None:
        self.get_logger().info("[ Camera.DEL ] Destroying Camera Node")

        ## Stop Pipeline
        if self.pipeline: self.pipeline.stop()
        return

    ### CameraNode: CALLBACK FUNCTIONS  ###

    ##########
    # Timer Callback Function
    def callback_timer(self) -> None:
        self.timer_counter += 1

        ## Collect Frames + Error Handling
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().error(f"[ Camera.CALLBACK_TIMER ] Error: {e}")
            return
        video_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        ## If Empty, Return
        if not video_frame: return
        if not depth_frame: return

        ## Collect Data
        image = video_frame.as_frame().get_data()
        np_image = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2RGB)

        #####################################
        # DONE: Implement Depth Frame      #
        #####################################


        #####################################
        # DONE: IMU -- Needs documentation  #
        #####################################
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        accl_frame = frames.first_or_default(rs.stream.accel)

        # if gyro_frame.is_motion_frame() : self.get_logger().warn("[ CALLBACK ] Collected Gyro Frame")
        # if accl_frame.is_motion_frame(): self.get_logger().warn("[ CALLBACK ] Collected Accel Frame")

                
        accl_data = accl_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        # self.get_logger().info(f"""
        #     Accel Frame:
        #         X: {accl_data.x}
        #         Y: {accl_data.y}
        #         Z: {accl_data.z}
        #     ------------------------
        # """)
        # self.get_logger().info(f"""
        #     Gyro Frame:
        #         X: {gyro_data.x}
        #         Y: {gyro_data.y}
        #         Z: {gyro_data.z}
        #     ------------------------
        # """)

        #####################################

        #####################################
        # DONE: IMU -- Publish Accel/Gyro   #
        #   Requires testing to ensure data #
        #   is published correctly          #
        #####################################

        ### Pack and Publish IMU Data
        ######################################
        data = AccelStamped()
        data.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()[1]
        data.header.stamp.sec = self.get_clock().now().seconds_nanoseconds()[0]
        data.header.frame_id = ""

        data.accel.linear.x = accl_data.x
        data.accel.linear.y = accl_data.y
        data.accel.linear.z = accl_data.z
        
        data.accel.angular.x = gyro_data.x
        data.accel.angular.y = gyro_data.y
        data.accel.angular.z = gyro_data.z

        self.pub_imu.publish(msg=data)
        ######################################


        ## Collect AprilTag Location
        ######################################
        np_grayscale = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2GRAY )
        result = self.detector.detect(np_grayscale)
        if len(result) == 0:
            # self.get_logger().warn("[ CameraNode.timer ] No tag detected")
            pass
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
        ######################################

        ## Generate Message
        ######################################
        rgb_msg = CompressedImage()
        depth_msg = Image()
        ######################################

        ## Populate RGB Message
        ######################################
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.format = "jpeg"
        ######################################


        ## Populate Depth Message
        ######################################
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.encoding = "16UC1"
        ######################################


        ## Pack Image Data
        ######################################
        rgb_msg.data = cv.imencode( ".jpg", np_image )[1].tobytes() # ChatGPT Suggested Fix
        ######################################

        ## Send Messages
        ######################################
        self.pub_rgb.publish(msg=rgb_msg)
        self.pub_depth.publish(msg=depth_msg)
        ######################################

        return

    ### CameraNode: Initiallizers       ###

    ##########
    # Initiallize AprilTag Detector
    def init_detector(self) -> None:
        self.get_logger().info("[ CameraNode.init_detector ] Initiallizing Detector")
        
        ## Initiallize the AprilTag detector object
        self.detector = apriltag.Detector()
        self.get_logger().info("[ CameraNode.init_detector ] Detector Initiallized")
        return

    ##########
    # Initiallize Timer
    def init_timer(self, period: float) -> None:
        self.get_logger().info(f"[ Camera.INIT_TIMER ] Starting Timer, Period={period}")
        
        ## Initiallize the Timer Object
        #   - Period set by period parameter
        #       - Determines Frame Rate: 
        #               24FPS == 0.042
        #               30FPS == 0.033
        #               60FPS == 0.017
        #   - Callback set to self.callback_timer method
        self.timer = self.create_timer(
            timer_period_sec=period,
            callback=self.callback_timer
        )
        self.get_logger().info("[ Camera.INIT_TIMER ] Timer Initiallized")
        return

    ##########
    # Initiallize Camera Pipeline
    def init_camera(self) -> None:
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting Camera")
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting IMU")

        ## Initiallize Camera Pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        ## Select, Configure, and Enable Streams in Pipeline configuration
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        ## Send Configuration to Pipeline
        self.pipeline.start( config )
        self.get_logger().info("[ Camera.INIT_CAMERA ] Camera Initiallized")
        self.get_logger().info("[ Camera.INIT_CAMERA ] IMU Initiallized")
        return

    ##########
    # Initiallize Compressed Image Publisher
    def init_node_publisher(self, rgb_topic_name: str, depth_topic_name: str, imu_topic_name: str, imu_accel_topic_name: str="", imu_gyro_topic_name: str="") -> None:
        ##########
        # Initiallize the RGB Camera Publisher
        #       Type == Compressed Image
        #       Topic == Specified in Function Call
        #       Quality of Service == 3 frames
        self.pub_rgb = self.create_publisher(
            msg_type = CompressedImage,
            topic = rgb_topic_name,
            qos_profile = 3
        )

        ##########
        # Initiallize the Depth Camera Publisher
        #       Type == Normal Image
        #       Topic == Specified in Function Call
        #       Quality of Service == 3 frames
        self.pub_depth = self.create_publisher(
            msg_type = Image,
            topic = depth_topic_name,
            qos_profile = 3
        )

        # DEPRECATED
        ##########
        # Initiallize the IMU Accelerometer Publisher
        #       Type == 32-bit Float Array
        #       Topic == Specified in Function Call
        #       Quality of Service == 5 frames
        # self.pub_accel = self.create_publisher(
            # msg_type    = Float32MultiArray,
            # topic       = imu_accel_topic_name,
            # qos_profile = 5    
        # )

        # DEPRECATED
        ##########
        # Initiallize the IMU Gyroscope Publisher
        #       Type == 32-bit Float Array
        #       Topic == Specified in Function Call
        #       Quality of Service == 5 frames
        # self.pub_gyro = self.create_publisher(
            # msg_type    = Float32MultiArray,
            # topic       = imu_gyro_topic_name,
            # qos_profile = 5
        # )

        ##########
        # Initiallize the Generallized IMU Publisher
        #       Type == Acceleration Frame w/ Time Stamp
        #       Topic == Specified in Function Call
        #       Quality of Service == 5 frames
        self.pub_imu = self.create_publisher(
            msg_type    = AccelStamped,
            topic       = imu_topic_name,
            qos_profile = 10
        )
        return

    ##########
    # Print Corners on AprilTag
    def overlay_tag_corners(self, np_image, tag):
        
        ## Determine of the tag passed is REALLY a tag:
        #   If NONE, then passed tag does not exist 
        if tag == None or np_image.size == 0:
            return np.zeros(1)

        ## Tag is real, what do we do now?
        else:
            # Collect Corners
            corners = tuple(map(list, tag.corners))

            # Print Corners to Image
            for i in range(4):
                cv.circle(np_image, ( int(corners[i][0]), int(corners[i][1]) ), 10, (255, 0, 0), 10)
            
            # Return Image (Find out if it is possible to modify by reference)
            return np_image

    ##########
    # Print Center on AprilTag
    def overlay_tag_center(self, np_image, tag):
        ## Determine of the tag passed is REALLY a tag:
        #   If NONE, then passed tag does not exist 
        if tag == None or np_image.size == 0:
            return np.zeros(1)

        ## Tag is real, what do we do now?
        else:
            # Collect Center
            center_X, center_Y = tuple(map(int, tag.center))

            # Print Center to Image
            cv.circle(np_image, (center_X, center_Y), 5, (255, 0, 255), 5)

            # Return Image (Find out if it is possible to modify by reference)
            return np_image        


#####################################
# Main Function
def main() -> int:
    rclpy.init()
    node = CameraNode(poll_period_sec=0.0166666666667)        
    try:
        rclpy.spin(node=node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\n\033[100m[ Main.MAIN ] Keyboard Interrupt Pressed", end='\033[0m\n')
    return 0

if __name__ == "__main__":
    # exit( main() )
    main()      # Maybe this will solve the Seg fault