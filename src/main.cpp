// Standard includes
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Socket and network handling
#include <arpa/inet.h>
#include <sys/socket.h>

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


// Global variables

static const std::string NODE_NAME = "kuka_rsi_ros_interface";
static const std::string PACKAGE_PATH = ros::package::getPath(NODE_NAME);

static const char* SERVER_ADDRESS = "172.31.1.146";
static const uint16_t SERVER_PORT = 49152;
static const unsigned int BUFFER_SIZE = 1024;


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
 * Returns a string with correct timestamp given some data received from the
 * robot controller and data that is to be sent.
 */
std::string updateTimestamp(std::string received_data,
        std::string data_to_send) {
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

    // Create socket file descriptor for UDP socket
    int server_fd;
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Failed to open socket.");
        exit(EXIT_FAILURE); 
    }

    // Enable address/port reuse
    int opt_reuse = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
            &opt_reuse, sizeof(opt_reuse)) < 0) {
        ROS_ERROR("Failed to set socket options.");
        exit(EXIT_FAILURE);
    }

    // Configure socket address
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    address.sin_port = htons(SERVER_PORT);
    
    // Bind socket to address/port
    if (bind(server_fd, (struct sockaddr *) &address,
            sizeof(address)) < 0) {
        ROS_ERROR("Failed to bind socket to %s:%u.", SERVER_ADDRESS,
            SERVER_PORT);
        exit(EXIT_FAILURE);
    }

    // Create buffer and length variable for socket data exchange
    char buffer[BUFFER_SIZE] = {0};
    int recvlen;

    ROS_INFO("Waiting to receive data from RSI on %s:%u...", SERVER_ADDRESS,
        SERVER_PORT);

    // Receive initial data from RSI
    recvlen = recvfrom(server_fd, buffer, BUFFER_SIZE, 0,
        (struct sockaddr *) &address, &addrlen);
    ROS_INFO("Received %d bytes.", recvlen);
    if (recvlen > 0) {
        buffer[recvlen] = 0;
        ROS_INFO("Received data:\n\"%s\"", buffer);
    }
    // TODO: Separate into function and give reply
    // ...

    // Set socket timeout for further data exchange
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if (setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
            sizeof(timeout)) < 0) {
        ROS_ERROR("Could not configure socket timeout for further data "
                  "exchange.");
    }

    // Main program loop
    while(ros::ok()) {
        // Call all callbacks waiting to be called
        ros::spinOnce();

        // Receive further data
        recvlen = recvfrom(server_fd, buffer, BUFFER_SIZE, 0,
            (struct sockaddr *) &address, &addrlen);
        if (recvlen == -1) {
            ROS_ERROR("Robot controller is no longer sending data. Closing "
                      "socket and exiting...");
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        ROS_INFO("Received %d bytes.", recvlen);
        buffer[recvlen] = 0;
        ROS_INFO("Received data:\n\"%s\"", buffer);

        // TODO...
        // buffer.
        // sendto(server_fd, buffer, strlen(buffer), 0,
        //     (struct sockaddr *) &address, addrlen);
    }

    ROS_INFO("Closing socket.");
    close(server_fd);

    return 0;

}
