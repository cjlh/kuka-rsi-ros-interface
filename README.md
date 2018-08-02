# kuka-rsi-ros-interface


## Overview

A ROS node for the manipulation of a KUKA robot arm via RSI 3.

Developed for ROS Kinetic and tested on Ubuntu 16.04.

**Keywords:** ros, kinetic, rsi, kuka


### License

The source code is released under the [GNU General Public License v3.0](kuka_rsi_ros_interface/LICENSE).

**Author: Caleb Hamilton (cjlhamilton@gmail.com)  
Maintainer: Caleb Hamilton (cjlhamilton@gmail.com)  
Status: Early development**


### Thanks to...

- Dalton Nuclear Institute (http://www.dalton.manchester.ac.uk/) for funding this project and granting permission to publish as open source;
- Eren Sezener and Osman Kaya from the Ozyegin University Robotics Laboratory for helping me to understand the RSI protocol through their work on the *KUKA RSI-3 Communicator* project (https://github.com/erensezener/kuka-rsi3-communicator).


## Installation

### Dependencies
[to do]

### Installation on robot controller 
1. From the `krc_files` directory, copy the file `RSI_Ethernet.src` to the directory `KRC:\R1\Program` on the robot controller
2. Copy the remaining files from the `krc_files` directory to the directory `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` on the robot controller (*Expert* user group is required)

### Building
1. Copy the directory `kuka_rsi_ros_interface` from `catkin_ws/src` to your catkin workspace source directory (i.e. `~/catkin_ws/src`)
2. Build the packages using catkin (Note: tested with catkin_tools only)


## Usage

### Configuration
[to do]

### Launching
[to do]


## Nodes
[to do]


## To-do

- [ ] Add yaml config files for IP, port, etc.
