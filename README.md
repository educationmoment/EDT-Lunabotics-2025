<h1>EDT-Lunabotics-2025</h1>

<img src="https://github.com/user-attachments/assets/6e77045f-b1a8-47ea-ad5b-2ae286bfaef3" style="width: 400px" alt="EDT-Logo"></img>

<hr>
<h2>Packages</h2>
    <ul>
        <li>aegis_bringup</li>
        <li>controller_pkg</li>
        <li>vision_pkg</li>
        <li>webgui_pkg</li>
    </ul>

<h2>Description</h2>
<hr>
<p>This is a GitHub repository created for the University of Illinois Chicago Engineering Design Team
for the NASA Lunabotics 2025 competition.</p>

    
<h2>Installation</h2>
<hr>
<h3><strong>(Optional)</strong> Using Docker</h3>
<p>Pull and run the ROS2 Humble Docker Image</p>

    docker run -it --network=host --privileged -v /dev/:/dev/ --name=ros-workspace ros:humble-ros-base

<p>Active and stopped containers can be listed using</p>

    docker ps --all

<p>If the container <em>ros-workspace</em> is already running, checked with the command above, then
you can either reattach to it using</p>

    docker start ros-workspace && docker attach ros-workspace

<p>OR remove the container using</p>

    docker rm ros-workspace

<p>Note: Removing the container means the entire container must be reinstalled; however, the ros-humble-base image remains.</p>

<h3>Clone this Github Repo</h3>

    git clone git@github.com:Cdiede2/EDT-Lunabotics-2025.git && cd EDT-Lunabotics-2025/

<p>From within a computer running Ubuntu 22.04, run <em>install.sh</em> script with root privileges using</p>
    
    sudo echo install.sh | bash

<p>The script periodically asks for user input and was designed to run inside a Docker container.
It updates the system, installs the Robotics Operating System (ROS2 Humble), and several dependencies.</p>

<h2>Setup</h2>
<hr>
<h3>Build the Project</h3>
<p>From within the workspace, run the following to build the entire project.</p>

    colcon build

<p> SparkCAN and Control Area Networks (CAN) will require the setup of the CAN interface on the host system.
A CAN interface should be immediately visible upon connecting the USB-to-CAN adapter. It may initially show as <strong>DOWN</strong>.
Configure the CAN interface using</p>

    sudo ip link set can0 up type can bitrate 1000000

<p>At this point, launch the robot using</p>

    ros2 launch aegis_bringup aegis_bringup.launch
    
