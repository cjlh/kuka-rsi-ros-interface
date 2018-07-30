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
    TiXmlDocument doc(xml_file_path.c_str());
    bool is_xml_loaded = doc.LoadFile();

    if (is_xml_loaded) {
        printf("Loaded RSI XML template file");
    } else  {
        ROS_ERROR("Failed to load RSI XML template file.");
        exit(EXIT_FAILURE);
    }

    // Set up socket
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    char buffer[BUFFER_SIZE] = {0};
    char hello[] = "Hello from server";

    // Creating socket file descriptor
    // AF_INET for IPv4
    // SOCK_DGRAM for UDP
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Failed to open socket.");
        exit(EXIT_FAILURE); 
    }

    // Forcefully attaching socket to port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
            &opt, sizeof(opt))) {
        ROS_ERROR("Failed to set socket options.");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    address.sin_port = htons(SERVER_PORT);
    
    // Forcefully attaching socket to port
    if (bind(server_fd, (struct sockaddr *) &address,
            sizeof(address)) < 0) {
        ROS_ERROR("Failed to bind socket to %s:%u.", SERVER_ADDRESS, SERVER_PORT);
        exit(EXIT_FAILURE);
    }

    ROS_INFO("Waiting to receive data.");

    // Main program loop
    while(ros::ok()) {
        // Call all callbacks waiting to be called
        ros::spinOnce();

        // Receive
        int recvlen = recvfrom(server_fd, buffer, BUFFER_SIZE, 0,
            (struct sockaddr *) &address, &addrlen);
        printf("received %d bytes\n", recvlen);
        if (recvlen > 0) {
            buffer[recvlen] = 0;
            printf("received message: \"%s\"\n", buffer);
        }

        // TODO...
        // buffer.
        // sendto(server_fd, buffer, strlen(buffer), 0, (struct sockaddr *) &address, addrlen);
    }

    ROS_INFO("Closing socket.");
    close(server_fd);

    return 0;

}
