// Standard headers
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>

// ROS headers
#include <ros/ros.h>

// XML parsing
#include <tinyxml.h>

// Class header
#include "RsiCommunicator.h"

// Package classes
#include "KukaPose.h"


/*
 * TODO.
 */
RsiCommunicator::RsiCommunicator(const char* ip_address,
                                 uint16_t port,
                                 int buffer_size) {
    this->port = port;
    this->ip_address = ip_address;
    this->buffer_size = buffer_size;

    // Create socket file descriptor for UDP socket
    if ((this->server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Failed to open socket.");
        throw std::exception();
    }

    // Enable address/port reuse
    int opt_reuse = 1;
    if (setsockopt(this->server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
            &opt_reuse, sizeof(opt_reuse)) < 0) {
        ROS_ERROR("Failed to set socket options.");
        throw std::exception();
    }

    // Configure socket address
    this->addrlen = sizeof(this->in_address);
    this->in_address.sin_family = AF_INET;
    this->in_address.sin_addr.s_addr = inet_addr(ip_address);
    this->in_address.sin_port = htons(port);
    
    // Bind socket to address/port
    if (bind(this->server_fd, (struct sockaddr *) &(this->in_address),
             sizeof(in_address)) < 0) {
        ROS_ERROR("Failed to bind socket to %s:%u.", ip_address, this->port);
        throw std::exception();
    }
}


/*
 * Destructor - ensure socket is closed on destruction.
 */
RsiCommunicator::~RsiCommunicator() {
    closeSocket();
}


/*
 *
 */
bool RsiCommunicator::setSocketTimeout(long sec, long usec) {
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = usec;
    if (setsockopt(this->server_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                   sizeof(timeout)) < 0) {
        return false;
    }
    return true;
}


/*
 * TODO.
 */
void RsiCommunicator::initiate() {
    // TODO: implement ctrl+c interrupt
    ROS_INFO("Waiting to receive data from RSI on %s:%u...",
             this->ip_address.c_str(),
             this->port);

    // Receive initial data from RSI
    try {
        // TODO: get rid of this ugly code and implement interrupt instead as a
        // matter of priority
        receiveDataFromController();
    } catch (const std::exception &e) {
        throw;
    }

    // Set socket timeout for further data exchange
    bool is_timeout_set = setSocketTimeout(5, 0);

    if (!is_timeout_set)  {
        ROS_ERROR("Could not configure socket timeout for further data "
                  "exchange.");
    }
}


/*
 * TODO.
 */
void RsiCommunicator::closeSocket() {
    ROS_WARN("Closing socket.");
    close(this->server_fd);
}


/*
 * TODO: sort syntax styling
 */
TiXmlDocument RsiCommunicator::receiveDataFromController() {
    char buffer[this->buffer_size] = {0};
    int recvlen = recvfrom(this->server_fd,
                           buffer,
                           this->buffer_size,
                           0,
                           (struct sockaddr *) &(this->out_address),
                           &(this->addrlen));
    if (recvlen == -1) {
        ROS_ERROR("Could not receive data from robot controller (%s).",
                  strerror(errno));
        close(this->server_fd);
        throw std::exception();
    } else {
        ROS_INFO("Received %d bytes.", recvlen);
        buffer[recvlen] = 0;
        ROS_INFO("Received data:\n\"%s\"", buffer);
    }
    std::string response_str(buffer, this->buffer_size);

    // Parse response as XML.
    TiXmlDocument response_xml;
    response_xml.Parse(response_str.c_str(), 0, TIXML_ENCODING_UTF8);
    return response_xml;
}


/*
 * Returns a string with correct timestamp given some data received from
 * the robot controller and data that is to be sent.
 */
TiXmlDocument RsiCommunicator::updateMessageTimestamp(
        TiXmlDocument received_data,
        TiXmlDocument data_to_send) {
    // Read <IPOC> tag from received_data
    // Set <IPOD> tag in data_to_send to received_data value
    // Return new string

    // Get timestamp value
    std::string timestamp;
    //TiXmlElement* received_data_root = received_data.RootElement();
    //timestamp = received_data_root->Attribute("IPOC");

    TiXmlHandle received_data_handle(&received_data);
    TiXmlElement* received_ipoc_elem = 
        received_data_handle.FirstChild("Rob").Child("IPOC", 0).ToElement();

    if (!received_ipoc_elem) {
        ROS_ERROR("Received RSI XML data could not be parsed (no timestamp).");
        throw std::exception();
    }

    std::string ipoc_value = std::string(received_ipoc_elem->GetText());
    ROS_INFO("Updating timestamp for IPOC: %s.", ipoc_value.c_str());

    TiXmlHandle data_to_send_handle(&data_to_send);
    TiXmlText* to_send_ipoc_text = data_to_send_handle.FirstChild("Sen")
                                                      .Child("IPOC", 0)
                                                      .FirstChild()
                                                      .ToText();

    if (!to_send_ipoc_text) {
        ROS_ERROR("Response RSI XML data could not be parsed (no timestamp).");
        throw std::exception();
    }

    to_send_ipoc_text->SetValue(ipoc_value.c_str());

    return data_to_send;
}


/*
 * TODO.
 * TODO: Decide if function should accept a string instead of TiXmlDocument.
 */
bool RsiCommunicator::sendInstructionToController(TiXmlDocument instruction) {
    // Use a tinyxml printer to convert the instruction to a string
    TiXmlPrinter printer;
    printer.SetIndent("");
    printer.SetLineBreak("");
    instruction.Accept(&printer);

    const char* data_to_send = printer.CStr();

    // Store string in buffer
    int send_data = sendto(this->server_fd,
                           data_to_send,
                           sizeof(data_to_send),
                           0,
                           (struct sockaddr *) &(this->out_address),
                           this->addrlen);
    if (send_data == -1) {
        ROS_ERROR("Could not send data to robot controller (%s).",
                  strerror(errno));
        close(this->server_fd);
        return false;
    } else {
        ROS_INFO("Sent %lu bytes.", sizeof(data_to_send));
        ROS_INFO("Sent data:\n\"%s\"", data_to_send);
        return true;
    }
}


/*
 * TODO.
 */
KukaPose RsiCommunicator::getCurrentPosition() {
    return KukaPose(1, 1, 1, 1, 1, 1);
}


/*
 * TODO.
 */
bool RsiCommunicator::moveToPosition(KukaPose pose) {
    return true;
}