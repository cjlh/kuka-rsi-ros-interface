// Standard includes
#include <unistd.h>
// #include <stdlib.h>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// YAML parsing
// #include "yaml-cpp/yaml.h"

// XML parsing
#include <tinyxml.h>

// Package classes
#include "KukaRsiRosInterface.h"

// Settings
// TODO: Move into yaml file
namespace settings
{
    const std::string node_name = "kuka_rsi_ros_interface";
    const std::string package_path =
        ros::package::getPath("kuka_rsi_ros_interface_core");
    const char* server_address = "172.31.1.146";
    const uint16_t server_port = 49152;
    const int buffer_size = 1024;
}


/*
 * TODO.
 */
int main(int argc, char **argv) {
    // Initialise ROS node.
    ros::init(argc, argv, settings::node_name);
    ros::NodeHandle nh;

    // Load RSI XML template file using TinyXML.
    std::string xml_file_path = settings::package_path +
                                "/xml/DataTemplate.xml";
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
        KukaRsiRosInterface interface(settings::node_name,
                                      settings::server_address,
                                      settings::server_port,
                                      settings::buffer_size,
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
