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
#include "kuka_rsi_ros_interface/KukaPose.h"

// Services
#include "kuka_rsi_ros_interface/MoveToPose.h"
#include "kuka_rsi_ros_interface/GetPose.h"

// Package classes
#include "RsiCommunicator.h"


// Global variables

static const std::string NODE_NAME = "kuka_rsi_ros_interface";
static const std::string PACKAGE_PATH = ros::package::getPath(NODE_NAME);

static const char* SERVER_ADDRESS = "172.31.1.146";
static const uint16_t SERVER_PORT = 49152;
static const int BUFFER_SIZE = 1024;


/*
 * [to do]
 */
bool getPositionValuesCallback(
        kuka_rsi_ros_interface::GetPose::Request &request,
        kuka_rsi_ros_interface::GetPose::Response &response) {
    ROS_INFO("Position values request received.");
    return true;
}


/*
 *
 */
bool movetoPositionCallback(
        kuka_rsi_ros_interface::MoveToPose::Request &request,
        kuka_rsi_ros_interface::MoveToPose::Response &response) {
    ROS_INFO("Position movement request received.");
    return true;
}


/*
 * Returns a string with correct timestamp given some data received from
 * the robot controller and data that is to be sent.
 */
std::string updateMessageTimestamp(std::string received_data,
                                   std::string data_to_send) {
    // Read <IPOC> tag from received_data
    // Set <IPOD> tag in data_to_send to received_data value
    // Return new string
    return "TODO";
}


/*
 * Initialises the ROS node and performs the following:
 * 1. [to do]
 */
int main(int argc, char **argv) {
    // Initialise ROS node
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    // Create service for requesting current position
    ros::ServiceServer get_position_service =
        nh.advertiseService("get_position_values", getPositionValuesCallback);
    ROS_INFO("Service \"/get_position_values\" started.");

    // Create service for moving to a position
    ros::ServiceServer move_to_position_service =
        nh.advertiseService("move_to_position", movetoPositionCallback);
    ROS_INFO("Service \"/move_to_position\" started.");

    // Load RSI XML template file
    std::string xml_file_path = PACKAGE_PATH + "/xml/DataTemplate.xml";
    TiXmlDocument rsiXmlTemplate(xml_file_path.c_str());
    bool is_xml_loaded = rsiXmlTemplate.LoadFile();

    if (is_xml_loaded) {
        ROS_INFO("Loaded RSI XML template file.");
    } else  {
        ROS_ERROR("Failed to load RSI XML template file.");
        exit(EXIT_FAILURE);
    }

    RsiCommunicator *rsi_comm = new RsiCommunicator(SERVER_ADDRESS, SERVER_PORT,
                                                    BUFFER_SIZE);
    rsi_comm->initiate();

    // Main program loop
    while(ros::ok()) {
        // Call all callbacks waiting to be called
        ros::spinOnce();

        std::string response;
        try {
            response = rsi_comm->receiveDataFromController();
        } catch (const std::exception &e) {
            ROS_ERROR("Problem communicating with robot controller. "
                      "Exiting...");
            exit(EXIT_FAILURE);
        }

        // process message received from controller

        // return message to controller
        // rsi_comm->sendInstructionToController(instruction);
    }

    delete rsi_comm;

    return 0;

}
