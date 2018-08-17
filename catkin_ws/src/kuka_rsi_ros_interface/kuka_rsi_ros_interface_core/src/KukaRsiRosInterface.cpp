// Standard includes
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <math.h>

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
#include "KukaPose.h"


KukaRsiRosInterface::KukaRsiRosInterface(const char* ip_address,
                                         uint16_t port,
                                         int buffer_size,
                                         TiXmlDocument instruction_template) {
    try {
        this->rsi_comm = new RsiCommunicator(ip_address, port, buffer_size);
    } catch (const std::exception &e) {
        throw;
    }
    ROS_INFO("UDP socket bound successfully.");
    this->instruction_template = instruction_template;
}


KukaRsiRosInterface::~KukaRsiRosInterface() {
    delete rsi_comm;
}


bool KukaRsiRosInterface::getPositionValuesCallback(
        kuka_rsi_ros_interface_msgs::GetPose::Request &request,
        kuka_rsi_ros_interface_msgs::GetPose::Response &response) {
    ROS_INFO("Position values request received.");
    KukaPose pose = getCurrentPosition();
    response.pose.x = pose.getX();
    response.pose.y = pose.getY();
    response.pose.z = pose.getZ();
    response.pose.a = pose.getA();
    response.pose.b = pose.getB();
    response.pose.c = pose.getC();
    // TODO: catch exception if getCurrentPosition fails and return false.
    // need to handle exceptions in the function first before they can be
    // caught here.
    return true;
}


bool KukaRsiRosInterface::movetoPositionCallback(
        kuka_rsi_ros_interface_msgs::MoveToPose::Request &request,
        kuka_rsi_ros_interface_msgs::MoveToPose::Response &response) {
    ROS_INFO("Position movement request received.");
    // Create a KukaPose object from the requested pose values.
    KukaPose goal_pose(request.pose.x,
                       request.pose.y,
                       request.pose.z,
                       request.pose.a,
                       request.pose.b,
                       request.pose.c);
    // Move to the specified position.
    bool success = false;
    try {
        success = moveToPosition(goal_pose);
    } catch (const std::exception &e) {
        // `moveToPosition` throws an std::exception if the XML template
        // data cannot be read.
        response.success = false;
        response.message = "The RSI XML template data could not be read.";
        return success;
    }
    if (success) {
        response.success = true;
        response.message = "Successfully moved robot to specified position.";
    } else {
        response.success = false;
        response.message = "Failed to move robot to specified position.";
    }
    return success;
}


TiXmlDocument KukaRsiRosInterface::cloneTiXmlDocument(TiXmlDocument &original) {
    TiXmlPrinter printer;
    printer.SetIndent("");
    printer.SetLineBreak("");
    original.Accept(&printer);

    const char* xml_str = printer.CStr();

    TiXmlDocument clone;
    clone.Parse(xml_str);
    return clone;
}


KukaPose KukaRsiRosInterface::getPositionDataFromResponse(
        TiXmlDocument &response) {
    // Create a handle for accessing the XML data.
    TiXmlHandle response_handle(&response);

    // Get a pointer to the <RIst> cartesian position data element
    TiXmlElement* rist_elem = 
        response_handle.FirstChild("Rob").Child("RIst", 0).ToElement();

    if (!rist_elem) {
        ROS_ERROR("Received RSI XML data could not be parsed (no position "
                  "data).");
        throw std::exception();
    }

    // Get value of each positioning attribute.
    const double x = atof(rist_elem->Attribute("X"));
    const double y = atof(rist_elem->Attribute("Y"));
    const double z = atof(rist_elem->Attribute("Z"));
    const double a = atof(rist_elem->Attribute("A"));
    const double b = atof(rist_elem->Attribute("B"));
    const double c = atof(rist_elem->Attribute("C"));

    return KukaPose(x, y, z, a, b, c);
}


KukaPose KukaRsiRosInterface::getCurrentPosition() {
    TiXmlDocument response = this->rsi_comm->receiveDataFromController();
    // TODO: May need to respond here.
    // TODO: Handle exceptions.
    return getPositionDataFromResponse(response);
}


double KukaRsiRosInterface::calculateCoordinateAdjustmentValue(
        double current_val, double goal_val, double coord_threshold) {
    // Coordinate adjustment heuristics.
    const double base_adj     = 0.02;
    const double max_adj      = 0.1;
    const double diff_divisor = 850.0;

    double diff = goal_val - current_val;

    if (fabs(diff) > coord_threshold) {
        if (current_val < goal_val) {
            // Need to increase the value.
            return std::min(max_adj, base_adj + (diff / diff_divisor));
        } else {
            // Need to decrease the value.
            return std::max(-max_adj, -base_adj + (diff / diff_divisor));
        }
    } else {
        return 0.0;
    }
}


KukaPose KukaRsiRosInterface::getAdjustmentPose(KukaPose current_pose,
                                                KukaPose goal_pose) {
    // Specify how close the actual end coordinate must be to the specified
    // goal coordinate.
    const double coord_threshold = 0.5;
    const double angle_threshold = 0.1;
    const double max_angle_adj   = 0.04;
    const double near_angle_dist = 5.0;

    // Calculate adjustment for each coordinate value.
    // If a coordinate is sufficiently close, do not make any adjustment,
    // otherwise calculate the adjustment to make based on how close it is.
    
    double x_adj, y_adj, z_adj;

    if (fabs(goal_pose.getX() - current_pose.getX()) <= coord_threshold) {
        x_adj = 0.0;
    } else {
        x_adj = calculateCoordinateAdjustmentValue(current_pose.getX(),
                                                   goal_pose.getX(),
                                                   coord_threshold);
    }
    
    if (fabs(goal_pose.getY() - current_pose.getY()) <= coord_threshold) {
        y_adj = 0.0;
    } else {
        y_adj = calculateCoordinateAdjustmentValue(current_pose.getY(),
                                                   goal_pose.getY(),
                                                   coord_threshold);
    }

    if (fabs(goal_pose.getZ() - current_pose.getZ()) <= coord_threshold) {
        z_adj = 0.0;
    } else {
        z_adj = calculateCoordinateAdjustmentValue(current_pose.getZ(),
                                                   goal_pose.getZ(),
                                                   coord_threshold);
    }

    // Calculate adjustment for each rotation value using quaternions. To do
    // this convert the goal pose and current pose into quaternion
    // representations; it is then possible to divide the goal quaternion by the
    // current quaternion to obtain the transformation required to get from one
    // position to the other.
    //
    // In the resulting quaternion, the axes specify the proportion of the
    // quaternion's angle that must be rotated by over the X, Y, Z axes in order
    // to reach the goal position. We can then rotate around each axis using
    // this proportion as a multiplier of a base velocity. When the resulting
    // transformation's angle is 0 we know we have reached our goal orientation.
    //
    // Note that the (A, B, C) values KUKA uses are YPR and not RPY, so we take
    // the resulting Z proportion for X and vice versa.

    // Create quaternion from goal position.
    double y_goal = angles::from_degrees(goal_pose.getA());
    double p_goal = angles::from_degrees(goal_pose.getB());
    double r_goal = angles::from_degrees(goal_pose.getC());
    tf::Quaternion q_goal = tf::createQuaternionFromRPY(r_goal, p_goal, y_goal);

    // Create quaternion from current position.
    double y_curr = angles::from_degrees(current_pose.getA());
    double p_curr = angles::from_degrees(current_pose.getB());
    double r_curr = angles::from_degrees(current_pose.getC());
    tf::Quaternion q_curr = tf::createQuaternionFromRPY(r_curr, p_curr, y_curr);

    // Create the transformation quaternion by multiplying the goal
    // quaternion by the inverse of the current quaternion.
    // This is the same as dividing the goal quaternion by the current
    // quaternion.
    tf::Quaternion q_new = q_goal * q_curr.inverse();

    // Convert the resulting angle to degrees.
    double angle = angles::to_degrees(q_new.getAngle());

    // Calculate the adjustment for (A, B, C) based on the resulting
    // proportions.

    double a_adj, b_adj, c_adj;

    if (angle < angle_threshold || angle > (360 - angle_threshold)) {
        // If the required rotation angle is sufficiently small, consider the
        // current position "close enough".
        a_adj = 0.0;
        b_adj = 0.0;
        c_adj = 0.0;
    } else {
        // Otherwise calculate the rotation for each axis based on this angle.
        if (angle < near_angle_dist) {
            // Slow down max adjustments when rotation angle is small.
            a_adj = q_new.getAxis().z()
                    * max_angle_adj
                    * (angle / near_angle_dist);

            b_adj = q_new.getAxis().y()
                    * max_angle_adj
                    * (angle / near_angle_dist);

            c_adj = q_new.getAxis().x()
                    * max_angle_adj
                    * (angle / near_angle_dist);
        } else {
            // Otherwise keep max adjustment as set.
            a_adj = q_new.getAxis().z() * max_angle_adj;
            b_adj = q_new.getAxis().y() * max_angle_adj;
            c_adj = q_new.getAxis().x() * max_angle_adj;
        }

        // Ensure that the angle is in the interval [0, 180].
        // Depending on the RPY values given the angle may sometimes be >180, to
        // correct this we can subtract the angle from 360 and invert the axes,
        // giving the correct rotation in the opposite direction.
        
        if (angle > 180) {
            angle = 360 - angle;
            a_adj *= -1;
            b_adj *= -1;
            c_adj *= -1;
        }
    }

    // Return the result as a KukaPose object.
    return KukaPose(x_adj, y_adj, z_adj, a_adj, b_adj, c_adj);
}


bool KukaRsiRosInterface::moveToPosition(KukaPose goal_pose) {
    ROS_INFO("Moving to goal position: "
             "X: %2f, Y: %2f, Z: %2f, A: %2f, B: %2f, C: %2f",
             goal_pose.getX(), goal_pose.getY(), goal_pose.getZ(),
             goal_pose.getA(), goal_pose.getB(), goal_pose.getC());

    bool is_in_position = false;

    while (!is_in_position) {
        // Receive data from controller to get current pose, but do not yet
        // respond.
        TiXmlDocument response = this->rsi_comm->receiveDataFromController();
        KukaPose current_pose = getPositionDataFromResponse(response);

        // Calculate adjustment values from current pose to move towards the
        // goal pose.
        KukaPose adjustment_pose = getAdjustmentPose(current_pose, goal_pose);

        // Create movement instruction by modifying XML of instruction template.
        TiXmlDocument instruction =
            cloneTiXmlDocument(this->instruction_template);
        instruction =
            this->rsi_comm->updateMessageTimestamp(response, instruction);

        // Create a handle for accessing the XML data.
        TiXmlHandle instruction_handle(&instruction);

        // Get a pointer to the <RIst> cartesian position data element
        TiXmlElement* rkorr_elem = 
            instruction_handle.FirstChild("Sen").Child("RKorr", 0).ToElement();

        // Check XML data has been parsed correctly.
        if (!rkorr_elem) {
            ROS_ERROR("Template RSI XML data could not be parsed (no position "
                      "data).");
            throw std::exception();
        }

        // Set adjustment values in XML response.
        rkorr_elem->SetDoubleAttribute("X", adjustment_pose.getX());
        rkorr_elem->SetDoubleAttribute("Y", adjustment_pose.getY());
        rkorr_elem->SetDoubleAttribute("Z", adjustment_pose.getZ());
        rkorr_elem->SetDoubleAttribute("A", adjustment_pose.getA());
        rkorr_elem->SetDoubleAttribute("B", adjustment_pose.getB());
        rkorr_elem->SetDoubleAttribute("C", adjustment_pose.getC());

        // Send instruction to controller and re-loop
        bool success = this->rsi_comm->sendInstructionToController(instruction);

        if (!success) {
            // TODO: exception or message.
            return false;
        }

        // If no more adjustments are required, end the loop.
        if (adjustment_pose.isZero()) {
            is_in_position = true;
        }
    }

    ROS_INFO("Goal position has been reached.");
    return true;
}


bool KukaRsiRosInterface::keepAlive() {
    try {
        // Use default instruction as specified in the XML template.
        TiXmlDocument instruction =
            cloneTiXmlDocument(this->instruction_template);

        // Get response data from controller as XML
        TiXmlDocument response = this->rsi_comm->receiveDataFromController();

        // Update timestamp of instruction to the timestamp received from
        // the controller.
        instruction =
            this->rsi_comm->updateMessageTimestamp(response, instruction);

        // Send instruction to robot controller.
        bool send_data =
            this->rsi_comm->sendInstructionToController(instruction);

        return send_data;
    } catch (const std::exception &e) {
        return false;
    }
}


void KukaRsiRosInterface::run() {
    try {
        // Use default instruction as specified in the XML template.
        TiXmlDocument instruction =
            cloneTiXmlDocument(this->instruction_template);

        this->rsi_comm->initiate(instruction);
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

        bool is_alive = keepAlive();

        if (!is_alive) {
            ROS_ERROR("Problem communicating with robot controller.");
            throw std::exception();
        }
    }
}