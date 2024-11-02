#!/usr/bin/env bash
set -e
# Set the environment
source /opt/ros/foxy/setup.bash 
source /opt/intel/openvino_2021/bin/setupvars.sh

# Change to build directory
cd ws

# Parse command line arguments
CACHE="false"
while getopts "c" opt; do
    case ${opt} in
        c )
            CACHE="true"
            ;;
        \? )
            echo "Usage: cmd [-c]"
            exit 1
            ;;
    esac
done

if [ "$CACHE" != "true" ]; then
    
    rosws update

    #######
    #
    # START - Pull request specific changes
    # 

    # Update packages for PR's
    # https://github.com/aws-deepracer/aws-deepracer-inference-pkg/pull/4
    cd aws-deepracer-inference-pkg
    git fetch origin pull/5/head:tflite
    git checkout tflite
    cd ..

    # https://github.com/aws-deepracer/aws-deepracer-camera-pkg/pull/5
    cd aws-deepracer-camera-pkg
    git fetch origin pull/5/head:compressed-image
    git checkout compressed-image
    cd ..

    # https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg/pull/4
    cd aws-deepracer-interfaces-pkg
    git fetch origin pull/4/head:compressed-image
    git checkout compressed-image
    cd ..

    # https://github.com/aws-deepracer/aws-deepracer-sensor-fusion-pkg/pull/4
    cd aws-deepracer-sensor-fusion-pkg
    git fetch origin pull/4/head:compressed-image
    git checkout compressed-image
    cd ..

    # https://github.com/aws-deepracer/aws-deepracer-model-optimizer-pkg/pull/2
    cd aws-deepracer-model-optimizer-pkg
    git fetch origin pull/3/head:tflite
    git checkout tflite
    cd ..

    # Patch with aws-deepracer-ctrl-pkg.patch
    cd aws-deepracer-ctrl-pkg
    git apply ../files/aws-deepracer-ctrl-pkg.patch
    cd ..

    # Remove previous builds (gives clean build)
    rm -rf install build log

    # Resolve the dependencies
    rosdep install -i --from-path . --ignore-src --rosdistro foxy -y

    #
    # END - Pull request specific changes
    #
    #######

    # Update deepracer_launcher.py (fix an issue in the file)
    cp ../files/launch/deepracer_launcher.py ./aws-deepracer-launcher/deepracer_launcher/launch/deepracer_launcher.py

    # Turn off SW update
    sed -i "s/ENABLE_PERIODIC_SOFTWARE_UPDATE = True/ENABLE_PERIODIC_SOFTWARE_UPDATE = False/" aws-deepracer-systems-pkg/deepracer_systems_pkg/deepracer_systems_pkg/software_update_module/software_update_config.py

fi

# Build the core
colcon build --packages-up-to deepracer_launcher rplidar_ros

# Build the add-ons
colcon build --packages-up-to logging_pkg 

cd ..

set +e
echo "Done!"
