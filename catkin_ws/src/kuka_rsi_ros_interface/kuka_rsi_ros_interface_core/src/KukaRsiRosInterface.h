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

/**
 * Responsible for core program logic, including dictating which data is
 * received from and sent to the robot controller.
 *
 * Utilising an object of the `RsiCommunicator` class to facilitate
 * communication, this class interprets responses from the robot controller
 * to parse the robot arm's current position, and allows the arm to be
 * manipulated into any given valid position.
 *
 * This class also implements the node's ROS services for obtaining and
 * modifying the robot position.
 */

class KukaRsiRosInterface
{
    private:
        /**
         * An `RsiCommunicator` object used for sending data to and receiving
         * data from the robot controller.
         */
        RsiCommunicator *rsi_comm;

        /**
         * A TinyXML document object used to store the RSI XML template used for
         * RSI instructions.
         */
        TiXmlDocument instruction_template;

    public:
        /**
         * Object contructor.
         *
         * Creates the `rsi_comm` object used for communication with the robot
         * controller and stores the RSI XML template.
         *
         * @param ip_address The IP address of the host PC.
         * @param port The port of the host PC to bind to.
         * @param buffer_size The size of buffer to use for receiving data from
         *     the robot controller.
         * @param instruction_template The XML template to use for communication
         *     via RSI.
         */
        KukaRsiRosInterface(const char* ip_address,
                            uint16_t port,
                            int buffer_size,
                            TiXmlDocument default_command);
        
        /**
         * Object destructor.
         *
         * Ensures that the memory used to store the `rsi_comm` object is freed
         * on destruction.
         */
        ~KukaRsiRosInterface();

        /**
         * ROS service callback for the node's `/get_position_vales` service.
         *
         * Retrieves the current positioning values from the robot and returns
         * them in the service response.
         *
         * @param request The ROS service request object of the message type
         *     `GetPose` from the `kuka_rsi_ros_interface_msgs` package.
         * @param response The ROS service response object of the same message
         *      type.
         * @return True if the service executes successfully, false otherwise.
         */
        bool getPositionValuesCallback(
            kuka_rsi_ros_interface_msgs::GetPose::Request &request,
            kuka_rsi_ros_interface_msgs::GetPose::Response &response);

        /**
         * ROS service callback for the node's `/move_to_position` service.
         *
         * Moves the robot into a position specified in the service request.
         *
         * @param request The ROS service request object of the message type
         *     `MoveToPose` from the `kuka_rsi_ros_interface_msgs` package.
         * @param response The ROS service response object of the same message
         *      type.
         * @return True if the service executes successfully, false otherwise.
         */
        bool movetoPositionCallback(
            kuka_rsi_ros_interface_msgs::MoveToPose::Request &request,
            kuka_rsi_ros_interface_msgs::MoveToPose::Response &response);

        /**
         * Clones a given TinyXML document and returns the clone.
         *
         * @param original A TinyXML document object.
         * @return A clone of the `original` document.
         */
        TiXmlDocument cloneTiXmlDocument(TiXmlDocument &original);

        /**
         * Extracts the position data of the robot from a TinyXML object
         * containing the controller's RSI XML response.
         *
         * @param response A TinyXML document object containing XML data
         *     received from the robot controller via RSI.
         * @return A KukaPose object containing the robot position data.
         */
        KukaPose getPositionDataFromResponse(TiXmlDocument &response);

        /**
         * Returns the current position of the robot obtained via RSI.
         *
         * @return A KukaPose object containing the robot's current position
         *     data.
         */
        KukaPose getCurrentPosition();

        /**
         * Calculates the amount to adjust a given axis coordinate value by in
         * one RSI transmission.
         *
         * Uses the current value, goal value, and allowable margin-of-error to
         * determine how much to adjust the coordinate value by.
         *
         * @param current_val The current coordinate value.
         * @param goal_val The coordinate value to move towards.
         * @return The adjustment calculated for the given values.
         */
        double calculateCoordinateAdjustmentValue(double current_val,
                                                  double goal_val,
                                                  double dist_threshold);

        /**
         * Calculates the required adjustments for all position values to send
         * in one RSI transmission.
         *
         * @param current_pose The current robot pose.
         * @param goal_pose The robot pose to move towards.
         * @return A KukaPose object containing the adjustment values for each
         *     pose attribute.
         */
        KukaPose getAdjustmentPose(KukaPose current_pose, KukaPose goal_pose);

        /**
         * Moves the robot from its current position to a specified goal
         * position.
         *
         * @param goal_pose The robot pose to move into.
         * @return True if the robot is successfully moved into the goal pose,
         *     false otherwise.
         */
        bool moveToPosition(KukaPose goal_pose);

        /**
         * Rotates the end effector (axis 6) of the robot a specified number of
         * degrees.
         *
         * @param rotation_degrees A number of degrees through which to rotate
         *     the robot's end effector ([SPECIFY LIMITS HERE]).
         * @return True if the end effector is successfully rotated, false
         *     otherwise.
         */
        bool rotateEndEffector(double rotation_degrees);

        /**
         * Keeps RSI communication alive by receiving the current data from the
         * robot and responding with the stored default XML template.
         *
         * This function simply copies the current controller timestamp from the
         * received XML response and responds with adjustment values of 0.
         *
         * @return True if communication with the robot controller is
         *     successful, and false otherwise.
         */
        bool keepAlive();

        /**
         * Begins the node's main program loop, which is executed until the node
         * is halted by the user, the ROS Master stops responding, or an error
         * occurs.
         *
         * This function initiates communication with the robot controller,
         * allowing further data to be received from and transmitted to the
         * controller.
         */
        void run();
};

#endif /* KukaRsiRosInterface */