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
#include "KukaPose.h"


/*
 * TODO.
 */
KukaRsiRosInterface::KukaRsiRosInterface(std::string node_name,
										 const char* ip_address,
										 uint16_t port,
                                 		 int buffer_size,
                                 		 TiXmlDocument instruction_template) {
    try {
		this->rsi_comm = new RsiCommunicator(ip_address, port, buffer_size);
    } catch (const std::exception &e) {
        throw;
    }
    ROS_INFO("UDP socket bound successfully.");
    this->node_name = node_name;
	this->instruction_template = instruction_template;
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


/*
 * TODO.
 */
bool KukaRsiRosInterface::movetoPositionCallback(
        kuka_rsi_ros_interface_msgs::MoveToPose::Request &request,
        kuka_rsi_ros_interface_msgs::MoveToPose::Response &response) {
    ROS_INFO("Position movement request received.");
    KukaPose target_pose(request.pose.x,
                         request.pose.y,
                         request.pose.z,
                         request.pose.a,
                         request.pose.b,
                         request.pose.c);
    bool success = moveToPosition(target_pose);
    if (success) {
        response.success = true;
        response.message = "Successfully moved robot to specified position.";
    } else {
        response.success = false;
        response.message = "Failed to move robot to specified position.";
    }
    return success;
}


/*
 * TODO.
 */
TiXmlDocument KukaRsiRosInterface::cloneTiXmlDocument(TiXmlDocument original) {
    TiXmlPrinter printer;
    printer.SetIndent("");
    printer.SetLineBreak("");
    original.Accept(&printer);

    const char* xml_str = printer.CStr();

    TiXmlDocument clone;
    clone.Parse(xml_str);
    return clone;
}


/*
 * TODO.
 */
KukaPose KukaRsiRosInterface::getPositionDataFromResponse(
        TiXmlDocument response) {
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


/*
 * TODO.
 */
KukaPose KukaRsiRosInterface::getCurrentPosition() {
    TiXmlDocument response = this->rsi_comm->receiveDataFromController();
    // TODO: May need to respond here.
    // TODO: Handle exceptions.
    return getPositionDataFromResponse(response);
}


/*
 * TODO.
 */
bool KukaRsiRosInterface::arePositionValuesWithinThreshold(double threshold,
                                                           KukaPose pose1,
                                                           KukaPose pose2) {
    return (abs(pose1.getX() - pose2.getX()) <= threshold
            && abs(pose1.getY() - pose2.getY()) <= threshold
            && abs(pose1.getZ() - pose2.getZ()) <= threshold
            && abs(pose1.getA() - pose2.getA()) <= threshold
            && abs(pose1.getB() - pose2.getB()) <= threshold
            && abs(pose1.getC() - pose2.getC()) <= threshold);
}


/*
 * TODO.
 */
double KukaRsiRosInterface::calculateCoordinateAdjustmentValue(
        double current_val, double target_val, double dist_threshold) {
    const double base_adj     = 0.02;
    const double max_adj      = 0.1;
    const double diff_divisor = 850.0;

    double diff = target_val - current_val;
    if (abs(diff) > dist_threshold) {
        if (current_val < target_val) {
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


/*
 * TODO.
 */
double KukaRsiRosInterface::calculateOrientationAdjustmentValue(
        double current_val, double target_val, double dist_threshold) {
    const double base_adj     = 0.025;
    const double max_adj      = 0.035;
    const double diff_divisor = 900.0;

    // Treat anything above 179.8 degrees and below -179.8 degrees as
    // equivalent. This will solve jittering issues around these values.
    if (179.9 <= abs(current_val) && 179.9 <= abs(target_val)) {
        ROS_WARN("Treating as 0");
        return 0.0;
    }

    // Annoyingly, the orientation values reported by the controller seem to
    // range from 0 to -180 and then jump to 180 down to 0 again. To get around
    // this, the value reported by the controller must be inverted so that
    // orientation values may be continuous between -180 and 180.
    //
    // This does need checked, but that's what it seems like is happening.
    // May need to switch to angle adjustments rather than coordinate
    // adjustments.

    if (current_val > 0) {
        current_val = 180 - current_val;
    } else if (current_val < 0) {
        current_val = -180 - current_val;
    }

    double diff = target_val - current_val;

    if (abs(diff) > dist_threshold) {
        if (current_val < target_val) {
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


/*
 * TODO.
 * TODO: Handle exceptions.
 */
bool KukaRsiRosInterface::moveToPosition(KukaPose target_pose) {
    // Define magic adjustment heuristics. <|:^0 /***
    const double dist_threshold = 0.5;

    ROS_INFO("Moving to target position: "
             "X: %2f, Y: %2f, Z: %2f, A: %2f, B: %2f, C: %2f",
             target_pose.getX(), target_pose.getY(), target_pose.getZ(),
             target_pose.getA(), target_pose.getB(), target_pose.getC());

    bool is_in_position = false;

    while (!is_in_position) {
        // Receive data from controller to get current pose, but do not yet
        // respond.
        TiXmlDocument response = this->rsi_comm->receiveDataFromController();
        KukaPose current_pose = getPositionDataFromResponse(response);

        // Check if current position is within threshold of target position;
        if (arePositionValuesWithinThreshold(dist_threshold,
                                             current_pose,
                                             target_pose)) {
            is_in_position = true;
        }

        // Calculate adjustment for each pose value.
        // If not within threshold distance of target pose, make adjustment,
        // otherwise adjust by 0.
        
        double x_adj = calculateCoordinateAdjustmentValue(current_pose.getX(),
                                                          target_pose.getX(),
                                                          dist_threshold);
        
        double y_adj = calculateCoordinateAdjustmentValue(current_pose.getY(),
                                                          target_pose.getY(),
                                                          dist_threshold);
        
        double z_adj = calculateCoordinateAdjustmentValue(current_pose.getZ(),
                                                          target_pose.getZ(),
                                                          dist_threshold);
        
        double a_adj = calculateOrientationAdjustmentValue(current_pose.getA(),
                                                           target_pose.getA(),
                                                           dist_threshold);
        
        double b_adj = calculateOrientationAdjustmentValue(current_pose.getB(),
                                                           target_pose.getB(),
                                                           dist_threshold);
        
        double c_adj = calculateOrientationAdjustmentValue(current_pose.getC(),
                                                           target_pose.getC(),
                                                           dist_threshold);

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

        if (!rkorr_elem) {
            ROS_ERROR("Template RSI XML data could not be parsed (no position "
                      "data).");
            throw std::exception();
        }

        // Get value of each positioning attribute.
        if (x_adj == 0.0) {
            rkorr_elem->SetAttribute("X", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("X", x_adj);
        }
        if (y_adj == 0.0) {
            rkorr_elem->SetAttribute("Y", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("Y", y_adj);
        }
        if (z_adj == 0.0) {
            rkorr_elem->SetAttribute("Z", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("Z", z_adj);
        }
        if (a_adj == 0.0) {
            rkorr_elem->SetAttribute("A", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("A", a_adj);
        }
        if (b_adj == 0.0) {
            rkorr_elem->SetAttribute("B", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("B", b_adj);
        }
        if (c_adj == 0.0) {
            rkorr_elem->SetAttribute("C", "0.0");
        } else {
            rkorr_elem->SetDoubleAttribute("C", c_adj);
        }

        // Send instruction to controller and re-loop
        bool success = this->rsi_comm->sendInstructionToController(instruction);

        if (!success) {
            // TODO: exception or message.
            return false;
        }
    }

    ROS_INFO("Target position has been reached.");
    return true;
}


/*
 * TODO.
 */
bool KukaRsiRosInterface::communicationStep(TiXmlDocument instruction) {
    try {
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


/*
 * TODO.
 */
void KukaRsiRosInterface::run() {
    try {
        this->rsi_comm->initiate(this->instruction_template);
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

        TiXmlDocument instruction =
            cloneTiXmlDocument(this->instruction_template);

        bool is_alive = communicationStep(instruction);

        if (!is_alive) {
            ROS_ERROR("Problem communicating with robot controller.");
            throw std::exception();
        }
    }
}