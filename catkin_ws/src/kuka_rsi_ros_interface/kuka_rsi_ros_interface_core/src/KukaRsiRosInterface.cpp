// Standard includes
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// YAML parsing
// #include "yaml-cpp/yaml.h"

// XML parsing
#include <tinyxml.h>

// Messages
#include <kuka_rsi_ros_interface_msgs/KukaPose.h>

// Services
#include <kuka_rsi_ros_interface_msgs/MoveToPose.h>
#include <kuka_rsi_ros_interface_msgs/GetPose.h>

// Class header
#include "KukaRsiRosInterface.h"

// Package classes
#include "RsiCommunicator.h"


/*
 * TODO.
 */
KukaRsiRosInterface::KukaRsiRosInterface(std::string node_name,
										 const char* ip_address,
										 uint16_t port,
                                 		 int buffer_size,
                                 		 TiXmlDocument default_command) {
    try {
		this->rsi_comm = new RsiCommunicator(ip_address, port, buffer_size);
    } catch (const std::exception &e) {
        throw;
    }
    ROS_INFO("UDP socket bound successfully.");
    this->node_name = node_name;
	this->default_command = default_command;
}


/*
 * Destructor - ensure socket is closed on destruction.
 */
KukaRsiRosInterface::~KukaRsiRosInterface() {
	delete rsi_comm;
}


/*
 * TODO.
 */
bool KukaRsiRosInterface::getPositionValuesCallback(
        kuka_rsi_ros_interface_msgs::GetPose::Request &request,
        kuka_rsi_ros_interface_msgs::GetPose::Response &response) {
    ROS_INFO("Position values request received.");
}


/*
 *
 */
bool KukaRsiRosInterface::movetoPositionCallback(
        kuka_rsi_ros_interface_msgs::MoveToPose::Request &request,
        kuka_rsi_ros_interface_msgs::MoveToPose::Response &response) {
    ROS_INFO("Position movement request received.");
}


void KukaRsiRosInterface::run() {
    try {
        rsi_comm->initiate(this->default_command);
    } catch (const std::exception &e) {
        ROS_ERROR("Problem initiating communication with robot controller.");
        throw;
    }

    // Main program loop
    while(ros::ok()) {
        // Call all callbacks waiting to be called. (Blocking) callbacks will 
        // use same RsiCommunicator object to interact with controler; once a
        // given callback has completed execution, control will return to this
        // loop to keep RSI alive.
        ros::spinOnce();

        // Continue receiving data from the robot controller and send default
        // response to keep RSI alive.

        // Get response data from controller as XML
        TiXmlDocument response;
        try {
            response = rsi_comm->receiveDataFromController();
        } catch (const std::exception &e) {
            ROS_ERROR("Problem communicating with robot controller.");
            throw;
        }

        // Update timestamp of default message to the timestamp received from
        // the controller.
        TiXmlDocument instruction;
        try {
            instruction =
                rsi_comm->updateMessageTimestamp(response,
                                                 this->default_command);
        } catch (const std::exception &e) {
            throw;
        }

        // Return message to controller to keep RSI alive.
        try {
            bool send_data = rsi_comm->sendInstructionToController(instruction);
        } catch (const std::exception &e) {
            ROS_ERROR("Problem communicating with robot controller.");
            throw;
        }
    }
}