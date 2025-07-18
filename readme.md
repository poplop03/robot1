# This repo is a ROS1 fully working autonomous AMR.
### The robot is equipped with these sensors and hardware interface:
- Bno055 IMU.
- ydlidar X3 lidar.
- hoverboard_driver controller.
- Kinect depth camera.
I strongly recommend using ROS2 as ROS1 is EOL, but you can still use this repo as template for a typical AMR setup, i hope it's support new learner into ROS.
### Check out these URL for original hardware interface:
- https://github.com/hoverboard-robotics/hoverboard-driver (hoverboard_driver ROS node)
- https://github.com/EFeru/hoverboard-firmware-hack-FOC (hoverboard_driver firmware)
- https://wiki.ros.org/ros_imu_bno055 (BNO055 IMU hardware interface)
- https://github.com/Shivam-Kumar-1/ros-noetic-kinectv1-setup (Kinect 360 ROS1 noetic setup)
- https://www.yahboom.net/study/YDLIDAR-X3 (ydalidar X3 tutorial from manufacturer) 

### install these package to run hoverboard controller or you can just run the 'installallpack.sh' for automatically install all of these package:
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

