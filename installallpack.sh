#!/bin/bash

set -e

echo "Updating package index..."
sudo apt update

echo "Installing ROS Noetic packages and dependencies..."

sudo apt install -y \
  ros-noetic-rosparam-shortcuts \
  qt5-default \
  ros-noetic-tf \
  ros-noetic-hardware-interface \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-controller-manager \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-laser-geometry

echo "All requested packages have been installed successfully."
