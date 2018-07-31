#ifndef RSICOMMUNICATOR_H
#define RSICOMMUNICATOR_H

// Standard headers
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>


// Set buffer size for server

/*
 * TODO.
 */
class RsiCommunicator
{
    private:
        /*
         * TODO.
         */
        std::string ip_address;

        /*
         * TODO.
         */
        uint16_t port;

        int buffer_size;

        int server_fd;

        struct sockaddr_in address;

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
        void initiate();

        /*
         * TODO.
         */
        void closeSocket();

        bool sendInstructionToController(const char* instruction);

        /*
         * TODO.
         */
        std::string receiveDataFromController();
};

#endif /* RsiCommunicator */