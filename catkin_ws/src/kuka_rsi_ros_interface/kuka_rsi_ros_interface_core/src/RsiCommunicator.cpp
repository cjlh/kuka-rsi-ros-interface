// Standard headers
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>

// ROS headers
#include <ros/ros.h>

// Class header
#include "RsiCommunicator.h"


/*
 * TODO.
 */
RsiCommunicator::RsiCommunicator(const char* ip_address, uint16_t port,
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
    this->addrlen = sizeof(this->address);
    this->address.sin_family = AF_INET;
    this->address.sin_addr.s_addr = inet_addr(ip_address);
    this->address.sin_port = htons(port);
    
    // Bind socket to address/port
    if (bind(this->server_fd, (struct sockaddr *) &(this->address),
             sizeof(address)) < 0) {
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
 * TODO: IMPLEMENT PROPER RAII DESIGN. (SO: 21511806)
 */
void RsiCommunicator::closeSocket() {
    ROS_WARN("Closing socket.");
    close(this->server_fd);
}


/*
 * TODO: sort syntax styling
 */
std::string RsiCommunicator::receiveDataFromController() {
    char buffer[this->buffer_size] = {0};
    int recvlen = recvfrom(this->server_fd, buffer, this->buffer_size, 0,
                           (struct sockaddr *) &(this->address),
                           &(this->addrlen));
    if (recvlen == -1) {
        ROS_ERROR("Robot controller is not sending data.");
        close(this->server_fd);
        throw std::exception();
    } else {
        ROS_INFO("Received %d bytes.", recvlen);
        buffer[recvlen] = 0;
        ROS_INFO("Received data:\n\"%s\"", buffer);
    }
    return std::string(buffer, this->buffer_size);
}

/*
 * TODO.
 */
bool RsiCommunicator::sendInstructionToController(const char* instruction) {
    return true;
}