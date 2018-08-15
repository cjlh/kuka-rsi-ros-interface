# kuka-rsi-ros-interface


## Overview

A ROS node for the manipulation of a KUKA robot arm via RSI 3.

Developed for ROS Kinetic and tested on Ubuntu 16.04. Tested using a KR C4 Compact controller running KUKA System Software 8.3 with KUKA.RobotSensorInterface 3.3 and a KR 3 R540 robot arm.

**Keywords:** ros, kinetic, rsi, kuka


### License

The source code is released under the [GNU General Public License v3.0](LICENSE).

**Author: Caleb Hamilton (cjlhamilton@gmail.com)  
Maintainer: Caleb Hamilton (cjlhamilton@gmail.com)  
Status: Early development**


### Thanks to:

- **The University of Manchester's** Dalton Nuclear Institute (http://www.dalton.manchester.ac.uk/) for funding this project and granting permission to publish as open source;
- Eren Sezener and Osman Kaya from the Ozyegin University Robotics Laboratory for helping me to understand the RSI protocol through their work on the *KUKA RSI-3 Communicator* project (https://github.com/erensezener/kuka-rsi3-communicator).


## Prerequisites

- The host PC running this software must be connected directly to the robot controller via Ethernet.
  - For the KR C4 Compact controller the Ethernet cable should be connected to the X66 port.
- The robot controller should be running KUKA System Software 8.3 with KUKA.RobotSensorInterface 3.3; different versions may be compatible but have not been tested.
- ROS Kinetic must be installed on the host PC, along with the packages listed in the 'ROS dependencies' section below.


## Installation

### ROS dependencies
[to do]

### Installation on robot controller
1.  Change user group to *Expert* on robot controller/KUKA smartPAD
    - `Main menu` -> `User group` -> `Expert`
2.  From the `krc_files` directory, copy the file `RSI_Ethernet.src` to the directory `KRC:\R1\Program` on the robot controller
    - E.g. using a USB stick
3.  Copy the remaining files from the `krc_files` directory to the directory `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` on the robot controller

### Building the `kuka_rsi_ros_interface` packages
1.  Copy the directory `kuka_rsi_ros_interface` from `catkin_ws/src` to your catkin workspace source directory (e.g. `~/catkin_ws/src`)
2.  Build the packages using catkin (Note: tested with catkin_tools only)
    - E.g. `$ catkin build`


## Usage and getting started

### Configuration
[to do]

### Launching
[to do]


## Nodes
[to do]


## To-do

- [ ] Add yaml config files for IP, port, etc.