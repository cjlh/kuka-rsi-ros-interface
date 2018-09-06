#ifndef RSICOMMUNICATOR_H
#define RSICOMMUNICATOR_H

// Standard headers
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>

// XML parsing
#include <tinyxml.h>

// Package classes
#include "KukaPose.h"


/**
 * TODO.
 */

class RsiCommunicator
{
    private:
        /**
         * The IP address to which the object's UDP socket will be bound to.
         */
        std::string ip_address;

        /**
         * The port to which the object's UDP socket will be bound to.
         */
        uint16_t port;

        /**
         * The size, in bytes, of buffer arrays to be used for receiving data
         * from the robot controller.
         */
        int buffer_size;

        /**
         * The file descriptor used for the object's UDP socket.
         */
        int server_fd;

        /**
         * A structure to store address information for the object's UDP socket.
         */
        struct sockaddr_in in_address;

        /**
         * A structure to store address information for the controller.
         */
        struct sockaddr_in out_address;

        /**
         * A convenience variable storing the length of the address structure
         * for easy reference.
         */
        socklen_t addrlen;

    public:
        /**
         * Constructor.
         */
        RsiCommunicator(const char* ip_address, uint16_t port, int buffer_size);
        
        /**
         * Destructor.
         */
        ~RsiCommunicator();

        /**
         * TODO.
         */
        bool setSocketTimeout(long sec, long usec);

        /**
         * TODO.
         */
        void closeSocket();

        /**
         * Returns a string with correct timestamp given some data received from
         * the robot controller and data that is to be sent.
         */
        TiXmlDocument updateMessageTimestamp(TiXmlDocument received_data,
                                             TiXmlDocument data_to_send);

        /**
         * TODO.
         */
        void initiate(TiXmlDocument initial_instruction);

        /**
         * TODO.
         */
        bool sendInstructionToController(TiXmlDocument instruction);

        /**
         * TODO.
         */
        TiXmlDocument receiveDataFromController();
};

#endif /* RsiCommunicator */