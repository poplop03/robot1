# This repo is a ROS1 fully working autonomous AMR.
### The robot is equipped with these sensors and hardware interface:
- Bno055 IMU.
- ydlidar X3 lidar.
- hoverboard_driver controller.
- Kinect depth camera.

#### I strongly recommend using ROS2 as ROS1 is EOL, but you can still use this repo as template for a typical AMR setup, I hope it's support new learner into ROS.
- The robot the data from robot (acquired with rosbag) can be used to offline SLAM via cartographer.
- The depth camera act like a high frequency 3D lidar, you can change puplish rate in the launch file. But in navigation mode, the 3d camera is combined with 2d laser scan to create a more reliable 2d lidar data. (as depth camera give more detail of the environment and create a 2d projection on the map)
- Sensor fusion is handled by `robot_localization` package, check `/robot_localization/params/sensor_fuse_config.yaml` and `/robot_localization/launch/sensor_fuse.launch` for detail configuration. This package providing realtive position of the robot with the fusion of IMU and wheel odometry.
- I built a ROS UI with react. Make sure that the robot and the webserver is run on the same local network: 
https://github.com/poplop03/react-ui-ros

### Node function is listed below, it's might be helpful.
- Bno055_IMU: This is a hardware interface between IMU and the ROS system. 
The data format is defined in sensor_msgs/Imu.msg.
- hoverboard_driver: This is a hardware interface between the hoverboard driver 
which is an asynchronous node. It sends wheel odometry data, linear velocity, 
angular velocity under the format nav_msgs/Odometry.msg to ROS system. 
Simultaneously, it receives the velocity command in from any source that have 
access to the topic.
- kinect_360_launch: This is a hardware interface between the Kinect 360 depth 
camera, typically it will sending depth image to others node.
59
- ydlidar_ros_driver: This is a hardware interface between the ydlidarX3 laser 
scanner providing point cloud in the surounding environment.
- robot_description_v7: This is a robot description node, describing the robot 
coordinate system, for instance, the transformation between camera and the robot 
base body.
- rosbag: This node works as a data acquisition for the robot, including current 
robot position, speed, imu data, battery voltage, point cloud from lidar or even 
image from camera. The data later can be used for visualization, analyzation, an 
offline SLAM. The data output is writen under .bag extension.
- map_server: In this case the node is used to save the map created with the robot 
when it does real-time mapping session.
- robot_localization: this node take IMU, wheel odometry and robot description 
to estimate the current robot pose reference to the odom frame (this frame is used 
as a origin of the robot in 2D environment).
- laser_filter1, laser_filter2: As the lidar data is not available in some segment of 
scanning, we need to eliminate the certain segment that affecting the robot 
perception. For example, if the laser scanner see that “there is a object that is 5cm 
far from me” but in reality, it is the original robot’s part. The filter node take part 
in cutting out unnecesary data.
- ira_laser_tools: this node used to combine the filtered lidar angle into the only 
source of laser scan.
- kinect360_depth_to_laser_scan: this node convert the depth image from 
camera and turn it into the 2D laser scanner, after that we can use it to joint the 
laser group with ira_laser_tools.
- rgbd_to_pcl: this node is transforming the dense image data in to lighter 3D 
point cloud that can be use for cartographer 3D SLAM.
- ROS_UI: this is the web interface interacting with user, for saving internet 
bandwith, after 1 second of not registering different velocity command it stop 
sending data over internet.
- cmd_web_relay: this node keep the last message of command velocity from UI 
and publish the command at steady 20hz.
- gmapping: This node listen to laser scan, in this case, it is the joined laser from 
ira_laser_tools. It subcribes to odometry topic coming from robot_localization
which is describing the robot current position referenced to the origin. It 
simultaneously localization and creating the map of the current environment.
- ros_bridge_server: this node is a standard package of ROS, it is a transport layer 
from web-based application protocol like websocket to ROS system. It’s a 
translator from JSON data to ROS specific message.
- Navigation_stack: This node is a combination of a set of planning algorithms 
with the input is the target position, the current estimated robot pose in relative 
to the map frame, the map is loaded in to the stack through map_server
- save_pose_send_goal: This node exposing 2 service and a parameters:
-   ~/save_pose_send_goal/save_pose: Saving the current pose of the robot in the 
map.
-   ~/save_pose_send_goal /send_goal : For sending goal to a predefined position.
-   ~/save_pose_send_goal/list_poses: This services is triggered by the webUI for 
listing out all current predefined location.
-   ~/save_pose_send_goal/target_name: The webUI will input the name for the 
target that stamped to the coordination.
![ROS node in mapping mode](/docs/pics/mapping_mode.png)
![ROS node in navigation mode](/docs/pics/nav_mode.png)

### Check out these URL for original hardware interface:
- https://github.com/hoverboard-robotics/hoverboard-driver (hoverboard_driver ROS node)
- https://github.com/EFeru/hoverboard-firmware-hack-FOC (hoverboard_driver firmware)
- https://wiki.ros.org/ros_imu_bno055 (BNO055 IMU hardware interface)
- https://github.com/Shivam-Kumar-1/ros-noetic-kinectv1-setup (Kinect 360 ROS1 noetic setup)
- https://www.yahboom.net/study/YDLIDAR-X3 (ydlidar X3 tutorial from manufacturer) 

### install these package to run hoverboard controller or you can just run the `installallpack.sh` for automatically install all of these package:

```
sudo apt install ros-noetic-rosparam-shortcuts
sudo apt install qt5-default
sudo apt install ros-noetic-tf
sudo apt install ros-noetic-hardware-interface
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt install ros-noetic-controller-manager
sudo apt install ros-noetic-cv-bridge
sudo apt install ros-noetic-image-transport
sudo apt install ros-noetic-laser-geometry
sudo apt install ros-noetic-ros-controllers
```

