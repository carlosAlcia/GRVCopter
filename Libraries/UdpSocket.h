// Created by Carlos Álvarez Cía 2023

#pragma once
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <string.h>
#include "Messages_GRVCopter.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>


using namespace std;

namespace UDP
{

#define PORT 5500
    class UDP_Socket
    {

    private:
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in remote_adr{};
        socklen_t size = sizeof(remote_adr);
        struct sockaddr_in remote_adr_reply{};

        socklen_t size_reply;
        struct sockaddr_in remote_adr_aux{};
        socklen_t size_reply_aux;
        bool saved_as_reply = false;

    public:
        UDP_Socket(char *ip)
        {
            struct sockaddr_in adr
            {
            };
            adr.sin_family = AF_INET;
            adr.sin_addr.s_addr = INADDR_ANY;
            adr.sin_port = htons(0);
            bind(sock, (struct sockaddr *)&adr, sizeof(adr));
            memset(&remote_adr, 0, sizeof(remote_adr));
            remote_adr.sin_family = AF_INET;
            inet_aton(ip, &remote_adr.sin_addr);
            remote_adr.sin_port = htons(PORT);

            print_port();
        }

        int Receive(char *buffer)
        {
            return recvfrom(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0, (struct sockaddr *)&remote_adr_aux, &size_reply_aux);
        }

        void Send(char *buffer)
        {
            sendto(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0, (struct sockaddr *)&remote_adr, size);
        }

        void SendReply(char *buffer)
        {
            if (saved_as_reply)
            {
                sendto(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0, (struct sockaddr *)&remote_adr_reply, size_reply);
            }
        }

        void Ping()
        {
            sendto(sock, "ping", sizeof("ping"), 0, (struct sockaddr *)&remote_adr, size);
        }

        void SaveAsReplyAddr()
        {
            remote_adr_reply = remote_adr_aux;
            size_reply = size_reply_aux;
            saved_as_reply = true;
            int a = 0;
        }

    private:
        void print_port()
        {
            struct sockaddr_in adr{};
            socklen_t local_sinlen = sizeof(adr);
            getsockname(sock, (struct sockaddr *)&adr, &local_sinlen);
            uint16_t port;
            port = htons(adr.sin_port);
            printf("Socket Port: %d\n", port);
        }
        };
    }