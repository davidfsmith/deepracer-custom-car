# Building the Software Packages

There are many ways to build the software packages. The easiest is to use the VS.Code Dev Container configurations that are included in the repository. 

## VS.Code Development Container Approach

- Ubuntu 20.04 (AMD64) + ROS2 Foxy for building packages for original DeepRacer
- Ubuntu 22.04 (AMD64) + ROS2 Humble
- Ubuntu 22.04 (ARM64) + ROS2 Humble for Raspberry Pi4.

Opening VS.Code will prompt you if you want to open the DevContainer. It will require Docker to be installed on your computer, Windows or Linux should be supported. To keep the bash history between sessions run `docker volume create deepracer-ros-bashhistory` before starting the container.

Running the ARM64 container on a normal x86 computer will require enabling cross-compilation in Docker:

        docker run --privileged --rm tonistiigi/binfmt --install all

The cross-compiled container uses qemu and is quite a bit slower than a native container.

Once DevContainer is runnning (first build might take a while) then packages can be installed with:

        ./build_scripts/build-core.sh

DEB packages can be created with:

        ./build_scripts/build-packages.sh

The packages will be created in the `dist/` folder.

## Dependencies

During installation a set of external packages are downloaded in the `src/` directory. These are primarily the official AWS DeepRacer packages as well as their dependencies, which will be patched as part of the build process.

| Package | Description |
|---------|-------------|
| [aws-deepracer-camera-pkg](https://github.com/aws-deepracer/aws-deepracer-camera-pkg.git) | Camera package for AWS DeepRacer |
| [aws-deepracer-ctrl-pkg](https://github.com/aws-deepracer/aws-deepracer-ctrl-pkg.git) | Control package for AWS DeepRacer |
| [aws-deepracer-device-info-pkg](https://github.com/aws-deepracer/aws-deepracer-device-info-pkg.git) | Device info package for AWS DeepRacer |
| [aws-deepracer-i2c-pkg](https://github.com/aws-deepracer/aws-deepracer-i2c-pkg.git) | I2C package for AWS DeepRacer |
| [aws-deepracer-inference-pkg](https://github.com/aws-deepracer/aws-deepracer-inference-pkg.git) | Inference package for AWS DeepRacer |
| [aws-deepracer-interfaces-pkg](https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg.git) | Interfaces package for AWS DeepRacer |
| [aws-deepracer-model-optimizer-pkg](https://github.com/aws-deepracer/aws-deepracer-model-optimizer-pkg.git) | Model optimizer package for AWS DeepRacer |
| [aws-deepracer-navigation-pkg](https://github.com/aws-deepracer/aws-deepracer-navigation-pkg.git) | Navigation package for AWS DeepRacer |
| [aws-deepracer-sensor-fusion-pkg](https://github.com/aws-deepracer/aws-deepracer-sensor-fusion-pkg.git) | Sensor fusion package for AWS DeepRacer |
| [aws-deepracer-servo-pkg](https://github.com/aws-deepracer/aws-deepracer-servo-pkg.git) | Servo package for AWS DeepRacer |
| [aws-deepracer-status-led-pkg](https://github.com/aws-deepracer/aws-deepracer-status-led-pkg.git) | Status LED package for AWS DeepRacer |
| [aws-deepracer-systems-pkg](https://github.com/aws-deepracer/aws-deepracer-systems-pkg.git) | Systems package for AWS DeepRacer |
| [aws-deepracer-usb-monitor-pkg](https://github.com/aws-deepracer/aws-deepracer-usb-monitor-pkg.git) | USB monitor package for AWS DeepRacer |
| [aws-deepracer-webserver-pkg](https://github.com/aws-deepracer/aws-deepracer-webserver-pkg.git) | Webserver package for AWS DeepRacer |
| [async_web_server_cpp](https://github.com/GT-RAIL/async_web_server_cpp.git) | Async web server for C++ |
| [web_video_server](https://github.com/RobotWebTools/web_video_server.git) | Web video server |
| [rplidar_ros](https://github.com/Slamtec/rplidar_ros.git) | RPLIDAR ROS package |
| [aws-deepracer-launcher](https://github.com/aws-deepracer/aws-deepracer-launcher.git) | Launcher package for AWS DeepRacer |
| [larsll-deepracer-logging](https://github.com/larsll/larsll-deepracer-logging.git) | Logging package for AWS DeepRacer |
