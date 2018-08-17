#ifndef KUKARSIROSINTERFACE_H
#define KUKARSIROSINTERFACE_H

// Standard includes
#include <string>

// Include ROS?
#include <ros/ros.h>

// XML parsing
#include <tinyxml.h>

// Services
#include <kuka_rsi_ros_interface_msgs/MoveToPose.h>
#include <kuka_rsi_ros_interface_msgs/GetPose.h>

// Package classes
#include "RsiCommunicator.h"

/*
 * TODO.
 */
class KukaRsiRosInterface
{
    private:
        /*
         * TODO.
         */
        std::string node_name;

        /*
         * TODO.
         */
        RsiCommunicator *rsi_comm;

        /*
         * TODO.
         */
        TiXmlDocument instruction_template;

    public:
        /*
         * Constructor.
         */
        KukaRsiRosInterface(std::string node_name,
                            const char* ip_address,
                            uint16_t port,
                            int buffer_size,
                            TiXmlDocument default_command);
        
        /*
         * Destructor.
         */
        ~KukaRsiRosInterface();

        /*
         * TODO.
         */
        bool getPositionValuesCallback(
            kuka_rsi_ros_interface_msgs::GetPose::Request &request,
            kuka_rsi_ros_interface_msgs::GetPose::Response &response);

        /*
         * TODO.
         */
        bool movetoPositionCallback(
            kuka_rsi_ros_interface_msgs::MoveToPose::Request &request,
            kuka_rsi_ros_interface_msgs::MoveToPose::Response &response);

        /*
         * TODO.
         */
        TiXmlDocument cloneTiXmlDocument(TiXmlDocument original);

        /*
         * TODO.
         */
        KukaPose getPositionDataFromResponse(TiXmlDocument response);

        /*
         * TODO.
         */
        KukaPose getCurrentPosition();

        /*
         * TODO.
         */
        double calculateCoordinateAdjustmentValue(double current_val,
                                                  double target_val,
                                                  double dist_threshold);

        /*
         * TODO.
         */
        KukaPose getAdjustmentPose(KukaPose current_pose, KukaPose goal_pose);

        /*
         * TODO.
         */
        bool moveToPosition(KukaPose target_pose);

        /*
         * TODO.
         */
        bool communicationStep(TiXmlDocument instruction);

        /*
         * TODO.
         */
        void run();
};

#endif /* KukaRsiRosInterface */