// Standard includes
#include <unistd.h>
// #include <stdlib.h>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// XML parsing
#include <tinyxml.h>

// Package classes
#include "KukaRsiRosInterface.h"


/**
 * Convert a C string to uint16_t.
 *
 * Used for reading the server port setting from the ROS parameter server.
 *
 * @param str C string containing the uint16_t value.
 * @return A uint16_t object parsed from the C string. 
 */
uint16_t str_to_uint16(const char *str) {
    uint16_t result;
    char *end;
    errno = 0;
    long tmp = strtol(str, &end, 10);
    if (errno || end == str || *end != '\0' || tmp < 0 || tmp > UINT16_MAX) {
        throw std::exception();
    }
    result = (uint16_t) tmp;
    return result;
}


/**
 * Main program function.
 *
 * Loads program configuration files, initialises node and begins program
 * execution.
 *
 * @param argc Number of strings pointed to by `argv`.
 * @param argv Argument vector containing command line arguments passed to the
 *             program.
 * @return Program execution success value.
 */
int main(int argc, char **argv) {
    // Initialise ROS node, including default node name.
    ros::init(argc, argv, "kuka_rsi_ros_interface");
    ros::NodeHandle nh;

    // Load configuration from parameter server - default values provided.

    // Load server address setting.
    std::string server_address;
    nh.param<std::string>("server/ip_address",
                          server_address,
                          "172.31.1.146");

    // Load buffer size setting.
    int buffer_size;
    nh.param<int>("server/buffer_size",
                  buffer_size,
                  1024);

    // Load server port setting.
    std::string server_port_str;
    nh.param<std::string>("server/port",
                          server_port_str,
                          "49152");
    uint16_t server_port;
    try {
        server_port = str_to_uint16(server_port_str.c_str());
    } catch (std::exception &e) {
        ROS_ERROR("Port setting could not be parsed from config file.");
        exit(EXIT_FAILURE);
    }

    ROS_INFO("Loaded server settings from parameter server");

    // Load RSI XML template file using TinyXML.
    std::string package_path =
        ros::package::getPath("kuka_rsi_ros_interface_core");
    std::string xml_file_path = package_path + "/xml/DataTemplate.xml";
    TiXmlDocument rsiXmlTemplate(xml_file_path.c_str());
    bool is_xml_loaded = rsiXmlTemplate.LoadFile();

    // Ensure XML file has been read correctly.
    if (is_xml_loaded) {
        ROS_INFO("Loaded RSI XML template file.");
    } else {
        ROS_ERROR("Failed to load RSI XML template file. Exiting...");
        exit(EXIT_FAILURE);
    }

    // TODO: implement keyboard interrupt
    try {
        // Create KukaRisRosInterface object, this will be responsible for
        // the core node logic.
        KukaRsiRosInterface interface(server_address.c_str(),
                                      server_port,
                                      buffer_size,
                                      rsiXmlTemplate);

        // Create service for requesting current position. The service accepts
        // an Empty message and returns a message of type KukaPose, containing
        // the X, Y, Z, A, B, C values of the robot position. For more details,
        // see the KukaPose class (not the message definition) in this
        // directory.
        ros::ServiceServer get_position_service =
            nh.advertiseService("get_position_values",
                                &KukaRsiRosInterface::getPositionValuesCallback,
                                &interface);
        ROS_INFO("Service \"/get_position_values\" started.");

        // Create service for moving to a given position. The service accepts a
        // message of type KukaPose (as described above) and replies with
        // a boolean "success" message and a string "message" message containing
        // a success confirmation if the service has executed successfully, or
        // an error message if not.
        ros::ServiceServer move_to_position_service =
            nh.advertiseService("move_to_position",
                                &KukaRsiRosInterface::movetoPositionCallback,
                                &interface);
        ROS_INFO("Service \"/move_to_position\" started.");

        // Begin node execution loop.
        interface.run();
    } catch (const std::exception &e) {
        ROS_FATAL("A non-recoverable error has occurred. Exiting...");
        exit(EXIT_FAILURE);
    }

    // Return 0 for successful execution.
    return 0;

}
