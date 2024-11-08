# DeepRacer Custom Car

## Description

DeepRacer Custom Car provides a set of features that will improve your DeepRacer car.

The repository is a merge of [deepracer-scripts](https://github.com/davidfsmith/deepracer-scripts) that contained improvements for the original car and [deepracer-pi](https://github.com/larsll/deepracer-pi) that ported DeepRacer to a Raspberry Pi4.

This repository contains a few different things:
 - Build and install scripts for a custom ROS2 software stack for DeepRacer.
 - Drawings and build instructions for building a DeepRacer compatible car based on a WLToys A979 and a Raspberry Pi4.
 - Utilities, e.g. to create a USB stick to flash the original DeepRacer.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Building a car with Raspberry Pi](#building-a-car-with-raspberry-pi)
- [Building the Software](#building-the-software)

## Features

The main features of the custom software stack is
- Performance improvement through using compressed image transport for main processing pipeline
- Inference using OpenVINO with Intel GPU (original DeepRacer), OpenVino with Myriad Neural Compute Stick (NCS), or Tensorflow Lite
- Model Optimizer caching, speeding up switching of models
- Capture in-car camera and inference results to a ROS Bag for logfile analysis
- UI tweaks and fixes
- Reduced log output from chatty nodes

Additionally there are OS level tweaks for the original DeepRacer and Raspberry Pi4 alike:
 - Original: Uninstall unused packages and stop unnecessary services
 - RPi: Minimal install, required packages only
 - Disable wifi power-save to avoid disconnects
 - Disable suspend

In the `utils/` folder there are utilities to create a USB flash stick for the original DeepRacer. See [documentation](docs/utilities.md).

## Installation

There are separate installation scripts for the original DeepRacer on Ubuntu 20.04/ROS2 Foxy, and the Raspberry Pi4 based car on Ubuntu 22.04/ROS2 Humble. Basic installation depends on pre-packages apt/deb packages, and does not require any packages to be compiled on the car itself.

For the original DeepRacer run:

        sudo install_scripts/aws-20.04/install-prerequisites.sh
        sudo install_scripts/aws-20.04/install-deepracer.sh

For the Raspberry Pi4:

        sudo install_scripts/rpi4-22.04/install-prerequisites.sh
        sudo install_scripts/rpi4-22.04/install-deepracer.sh

See also [building instructions](docs/raspberry_pi.md) for the Raspberry Pi4.

## Usage

Once installed the software will look very much like the original DeepRacer software.

The custom stack exposes the following arguments which can be changed through changing `/opt/aws/deepracer/start_ros.sh`.

| Argument | Default | Description | 
| -------- | ------- | ----------- |
| `camera_fps` | `30` | Number of camera frames per second, directly impacting how frequently the car takes new actions. |
| `camera_resize` | `True` | Does the camera resize from 640x480 to 160x120 at source. | 
| `inference_engine` | `TFLITE` | Inference engine to use (TFLITE or OV). |
| `inference_device` | `CPU` | Inference device to use, applicable to OV only (CPU, GPU or MYRIAD). |
| `logging_mode` | `usbonly` | Enable the logging of results to ROS Bag on USB stick. |
| `battery_dummy` | `False` | Use static dummy for battery measurements. |

Example custom `/opt/aws/deepracer/start_ros.sh`:
    
    source /opt/ros/foxy/setup.bash
    source /opt/aws/deepracer/lib.custom/setup.bash
    source /opt/intel/openvino_2021/bin/setupvars.sh
    ros2 launch deepracer_launcher deepracer_launcher.py camera_resize:=False camera_fps:=15 logging_mode:=Always

### Inference engine 

The different combinations of `inference_engine` and `inference_device` is not all compatible with the RPi4, and each option comes with pros and cons. The original car software only supports OpenVINO CPU.

| Type | Original | RPi4 | Comment |
|------|----------|------|---------|
| Tensorflow Lite CPU | Yes | Yes  | Default for RPi4
| OpenVINO CPU | Yes | No | Default for Original
| OpenVINO GPU | Yes | No | Reduced CPU load, model takes longer to load
| OpenVINO NCS2 / Myriad X | Yes | Yes | Reduced CPU load, model takes longer to load, requires NCS2 stick

The different modes have been tested for equivalency, and it is verified that they provide the same results (identical picture in -> identical action taken). Less than 1 per 1000 frames are differing, mainly due to the model not having a clear action, and several actions are having very similar probabilities.

### Logging

If you insert a USB stick or SD card with a folder called `logs/` then this will be automatically mounted, and when you start the car in autonomous mode then a ROS Bag file is created storing the inference results, which also contains the ROS messages.

Images are only captured if the autonomous mode is running ("Start Vehicle"). If stopped no images are stored. If the same model is restarted within 15 seconds the new images are appended to the same bag. After 15 seconds the bag is closed, and if restarted a new bag is created. If the selected model changes, the current bag is immediately closed, and a new one created once the car is started.

[larsll-deepracer-logging](https://github.com/larsll/larsll-deepracer-logging.git) is providing a script to convert the bag into a video, combining it with a grad-cam analysis to visualize what happens inside of the car.

## Building a car with Raspberry Pi

To create a DeepRacer compatible car with WLToys A979 and a Raspberry Pi read the [building instructions](docs/raspberry_pi.md).

## Building the Software

To build the software packages, either for the Raspberry Pi4 or for the original Deepracer [read the instructions](docs/build_sw.md).

