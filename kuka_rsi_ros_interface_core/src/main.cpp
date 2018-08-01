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
    // Initialise ROS node
    ros::init(argc, argv, settings::node_name);
    ros::NodeHandle nh;

    // Load RSI XML template file
    std::string xml_file_path = settings::package_path +
                                "/xml/DataTemplate.xml";
    TiXmlDocument rsiXmlTemplate(xml_file_path.c_str());
    bool is_xml_loaded = rsiXmlTemplate.LoadFile();

    if (is_xml_loaded) {
        ROS_INFO("Loaded RSI XML template file.");
    } else  {
        ROS_ERROR("Failed to load RSI XML template file. Exiting...");
        exit(EXIT_FAILURE);
    }

    // TODO: check program structure and implement keyboard interrupt
    try {
        KukaRsiRosInterface interface(settings::node_name,
                                      settings::server_address,
                                      settings::server_port,
                                      settings::buffer_size,
                                      rsiXmlTemplate);

        // Create service for requesting current position
        ros::ServiceServer get_position_service =
            nh.advertiseService("get_position_values",
                                &KukaRsiRosInterface::getPositionValuesCallback,
                                &interface);
        ROS_INFO("Service \"/get_position_values\" started.");

        // Create service for moving to a position
        ros::ServiceServer move_to_position_service =
            nh.advertiseService("move_to_position",
                                &KukaRsiRosInterface::movetoPositionCallback,
                                &interface);
        ROS_INFO("Service \"/move_to_position\" started.");

        interface.run();
    } catch (const std::exception &e) {
        ROS_FATAL("A non-recoverable error has occurred. Exiting...");
        exit(EXIT_FAILURE);
    }

    return 0;

}
