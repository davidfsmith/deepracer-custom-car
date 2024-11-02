#!/usr/bin/env bash
# 
# This script sets up the build prerequisites on an actual DeepRacer car. It ensures that all necessary packages are installed,
# detects the ROS (Robot Operating System) version, sets up the OpenVINO environment, and updates ROS dependencies.
#
# Steps performed by the script:
# 1. Update and upgrade all installed packages.
# 2. Install required pre-requisite packages: python3-websocket, python3-colcon-common-extensions, and python3-rosinstall.
# 3. Detect the installed ROS version (either "foxy" or "humble") and source the corresponding setup script.
# 4. Set up the OpenVINO environment by sourcing the appropriate setup script and installing GPU drivers if necessary.
# 5. Initialize and fix permissions for ROS dependencies using rosdep.
#
# Note: The script exits with an error message if an unsupported ROS or OpenVINO version is detected.

# Ensure all packages are up-to-date
sudo apt-get update
sudo apt-get upgrade -y

# Install additional pre-requisites
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

# Initialize ROS dependencies and fix permissions
sudo rosdep init
sudo rosdep fix-permissions
