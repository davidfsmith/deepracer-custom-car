# Source structure

## DeepRacer Packages

The following packages are part of the core DeepRacer software and the upstream versions are maintained by AWS.

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
| [aws-deepracer-launcher](https://github.com/aws-deepracer/aws-deepracer-launcher.git) | Launcher package for AWS DeepRacer |

## External Packages

External packages are mounted during the build process. Depending on the ROS version (Foxy, Humble or Jazzy) more or less of these are required.

| [async_web_server_cpp](https://github.com/GT-RAIL/async_web_server_cpp.git) | Async web server for C++ |
| [web_video_server](https://github.com/RobotWebTools/web_video_server.git) | Web video server |
| [rplidar_ros](https://github.com/Slamtec/rplidar_ros.git) | RPLIDAR ROS package |
| [camera_ros](https://github.com/christianrauch/camera_ros) | Camera package utilizing LibCamera (used for Raspberry Pi) |
| [larsll-deepracer-logging](https://github.com/larsll/larsll-deepracer-logging.git) | Logging package for AWS DeepRacer |

## Licenses

The repositories are all coming with a LICENSE file in the relevant subdirectory.