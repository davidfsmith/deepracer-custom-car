#!/usr/bin/env bash

# This script sets up the build prerequisites for an actual DeepRacer car, only basic tools are missing.

# Ensure all packages are installed
sudo apt-get update
sudo apt-get upgrade -y

# Install pre-requisites
sudo apt-get install -y python3-websocket python3-colcon-common-extensions python3-rosinstall

# Detect ROS version
if [ -f /opt/ros/foxy/setup.bash ]; then
    ROS_DISTRO="foxy"
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_DISTRO="humble"
    source /opt/ros/humble/setup.bash
else
    echo "Unsupported ROS version"
    exit 1
fi
echo "Detected ROS version: $ROS_DISTRO"

# Set the OpenVINO environment
if [ -f /opt/intel/openvino_2021/bin/setupvars.sh ]; then
    source /opt/intel/openvino_2021/bin/setupvars.sh
    sudo /opt/intel/openvino_2021/install_dependencies/install_NEO_OCL_driver.sh -y
elif [ -f /opt/intel/openvino_2022/setupvars.sh ]; then
    source /opt/intel/openvino_2022/setupvars.sh
else
    echo "Unsupported OpenVINO version"
    exit 1
fi

# Update ROS
sudo rosdep init
sudo rosdep fix-permissions
rosdep update --rosdistro $ROS_DISTRO
