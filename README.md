# kuka-rsi-ros-interface


## Overview

A ROS node for the manipulation of a KUKA robot arm via RSI 3.

This node advertises services for moving a KUKA robot arm into a desired position—specifying the X, Y, Z, A, B, C values for the robot's end effector—and for obtaining its current position.

This node for ROS Kinetic and tested on Ubuntu 16.04. Tested using a KR C4 Compact controller running KUKA System Software 8.3 with KUKA.RobotSensorInterface 3.3 and a KR 3 R540 robot arm.

**Keywords:** ros, kinetic, rsi, kuka


### License

The source code is released under the [GNU General Public License v3.0](LICENSE).

**Author: Caleb Hamilton (c.j.l.hamilton@gmail.com)  
Maintainer: Caleb Hamilton (c.j.l.hamilton@gmail.com)  
Status: In development**


### Thanks to:

- **The University of Manchester's** [Dalton Nuclear Institute](http://www.dalton.manchester.ac.uk/) for funding this project and granting permission to publish as open source;
- Eren Sezener and Osman Kaya from the Ozyegin University Robotics Laboratory for helping me to understand the RSI protocol through their work on their [KUKA RSI-3 Communicator](https://github.com/erensezener/kuka-rsi3-communicator) project.


## Prerequisites

- The host PC running this software must be connected directly to the robot controller via Ethernet.
  - For the KR C4 Compact controller the Ethernet cable should be connected to the X66 port.
- The robot controller should be running KUKA System Software 8.3 with KUKA.RobotSensorInterface 3.3; different versions may be compatible but have not been tested.
- ROS Kinetic must be installed on the host PC, along with any other dependencies listed in the section below.


## Installation

The following instructions assume you have already downloaded/cloned this repository onto your host PC. This can be done from the terminal using `$ git clone https://github.com/cjlh/kuka-rsi-ros-interface.git`.

### Installation on robot controller

1.  Change user group to *Expert* on robot controller/KUKA smartPAD
    - `Main menu` -> `User group` -> `Expert`
2.  From the `krc_files` directory, copy the file `RSI_Ethernet.src` to the directory `KRC:\R1\Program` on the robot controller
    - E.g. using a USB stick
3.  Copy the remaining files from the `krc_files` directory to the directory `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` on the robot controller

### Building from source

#### Dependencies

- [Robot Operating System (ROS)](http://www.ros.org/) Kinetic,
- [TinyXML](http://www.grinninglizard.com/tinyxml/) (included with ROS).

#### Building

1.  Copy the directory `kuka_rsi_ros_interface` from `catkin_ws/src` to your catkin workspace source directory (e.g. `~/catkin_ws/src`)
2.  Build the packages using catkin (Note: tested with catkin_tools only)
    - E.g.:
        1.  `$ cd ~/catkin_ws`
        2.  `$ catkin build`
        3.  `$ source devel/setup.bash`
    - Note that the `kuka_rsi_ros_interface` directory contains two packages: `kuka_rsi_ros_interface_core` and `kuka_rsi_ros_interface_msgs`; the `kuka_rsi_ros_interface_core` package is dependent upon the message and service types defined in `kuka_rsi_ros_interface_msgs`, and so they both must be built.


## Usage and getting started

### Configuration

The `kuka_rsi_ros_interface` node loads the configuration for various settings and parameters from the ROS Parameter Server. When using the included launch file, as detailed in the 'Launching' section below, these configuration settings are loaded onto the ROS parameter server from the `config.yaml` YAML file in the `kuka_rsi_ros_interface_core` package. This file can be found in the package's `config` directory.

To change the node configuration, you may modify the `config.yaml` file directly. This should be fairly intuitive, but if you have not used YAML before the [Complete Idiot's Introduction to YAML](https://github.com/Animosity/CraftIRC/wiki/Complete-idiot%27s-introduction-to-yaml) may be a good resource.

#### Changing IP address/port settings

If you wish to change the IP address and/or port settings in `config.yaml`, you must also change the same values in the `RSI_EthernetConfig.xml` file that is uploaded to the robot controller. For instructions on how to upload this file, see the 'Installation on robot controller' section above.

#### Default configuration

The default configuration file with included annotations is as follows:

```yaml
---
# Server settings.
server:
  # IP address to bind to on the host PC.
  ip_address: "172.31.1.146"
  # Port to bind to on the host PC.
  port: 49152
  # Buffer size used for receiving data from the robot controller.
  buffer_size: 1024

# Settings for adjusting the robot position.
adjustment_heuristics:
  # Heuristics for adjusting the X, Y, Z coordinates of the robot.
  coordinates:
    # Minimum coordinate adjustment per RSI instruction when moving into a given
    # position.
    base_adj: 0.02
    # Maximum coordinate adjustment per RSI instruction when moving into a given
    # position.
    max_adj: 0.1
    # Amount to divide the difference between a current coordinate value and a
    # target coordinate value by to add onto the `base_adj` value to obtain the
    # adjustment to send in a given RSI instruction for that coordinate value
    # (if not exceeding `max_adj`).
    diff_divisor: 850.0
    # Maximum distance of each of the robot's actual position coordinates from a
    # given target position's coordinates for the robot to be considered
    # in-position.
    threshold: 0.5
  # Heuristics for adjusting the A (yaw), B (pitch), C (roll) rotation values of
  # the robot.
  angles:
    # Maximum rotation value adjustment per RSI instruction when moving into a
    # given position.
    max_adj: 0.04
    # Angle between the robot's current position axes and the target position
    # axes below which the robot's movement slows down to allow for greater
    # movement accuracy as it approaches the target position.
    near_distance: 5.0
    # Maximum angle between the robot's current position axes and the target
    # position axes for the robot to be considered in-position.
    threshold: 0.1
```

### Launching

To launch the `kuka_rsi_ros_interface` node with the configuration file as above, simply run the following command in the terminal:

```
$ roslaunch kuka_rsi_ros_interface_core start.launch
```

This will load the contents of `config.yaml` to the ROS Parameter Server and launch the node (it will also launch the ROS Master if it is not already running).


### Usage

The `kuka_rsi_ros_interface` node advertises two services: `/move_to_position` and `/get_position_values`. The custom message type and service types for both of thse services are defined in the `kuka_rsi_ros_interface_msgs` package.

Both services utilise the `KukaPose` ROS message type, which is defined as follows:

```
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
```

#### Service 1: `/move_to_position`

The `/move_to_position` service uses the `MoveToPose` service type, which accepts a message of type `KukaPose`, containing each of the X, Y, Z, A, B, C values of the desired end effector position, and returns a boolean and a string. The service is defined as follows:

```
KukaPose pose
---
bool success
string message
```

The `success` boolean is returned as `true` if the robot is moved successfully, and `false` otherwise. If an error is encountered in moving the robot, the `message` string will contain a relevant error message, otherwise it will contain the message `"Successfully moved robot to specified position"`.

#### Service 2: `/get_position_values`

The `/get_position_values` service uses the `GetPose` service type, which accepts an empty message (of ROS standard type `std_msgs/Empty`) which is defined as follows:

```
---
KukaPose pose
```

The returned `KukaPose` message contains each of the X, Y, Z, A, B, C values of the robot's current end effector position.


## To-do

- [ ] Add yaml config files for IP, port, etc.
- [ ] Rename services to include namespace
