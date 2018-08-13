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


/*
 * TODO.
 */
class RsiCommunicator
{
    private:
        /*
         * The IP address to which the UDP socket will be bound to.
         */
        std::string ip_address;

        /*
         * The port to which the UDP socket will be bound to.
         */
        uint16_t port;

        /*
         * The size, in bytes, that will be used for buffer arrays.
         */
        int buffer_size;

        /*
         * The file descriptor used for the UDP socket.
         */
        int server_fd;

        /*
         * Structure to store address information for the UDP socket.
         */
        struct sockaddr_in in_address;

        /*
         * Structure to store address information for the controller.
         */
        struct sockaddr_in out_address;

        /*
         * Stores length of the address structure for easy reference.
         */
        socklen_t addrlen;

    public:
        /*
         * Constructor.
         */
        RsiCommunicator(const char* ip_address, uint16_t port, int buffer_size);
        
        /*
         * Destructor.
         */
        ~RsiCommunicator();

        /*
         * TODO.
         */
        bool setSocketTimeout(long sec, long usec);

        /*
         * TODO.
         */
        void initiate(TiXmlDocument initial_instruction);

        /*
         * TODO.
         */
        void closeSocket();

        /*
         * TODO.
         */
        TiXmlDocument updateMessageTimestamp(TiXmlDocument received_data,
                                             TiXmlDocument data_to_send);

        /*
         * TODO.
         */
        bool sendInstructionToController(TiXmlDocument instruction);

        /*
         * TODO.
         */
        TiXmlDocument receiveDataFromController();

        /*
         *
         */
        KukaPose getCurrentPosition();

        /*
         * TODO.
         */
        bool moveToPosition(KukaPose pose);
};

#endif /* RsiCommunicator */