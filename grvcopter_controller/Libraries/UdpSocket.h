//Created by Carlos Álvarez Cía 2023

#pragma once
#include <arpa/inet.h>    
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <string.h>
#include "Messages_GRVCopter.h"

using namespace std;

namespace UDP{
    
    #define PORT 5500
    class UDP_Socket {

        private:
            int sock = socket(AF_INET , SOCK_DGRAM , 0);
            struct sockaddr_in remote_adr{};
            socklen_t size = sizeof(remote_adr);


        public:
            UDP_Socket(char *ip){
                struct sockaddr_in adr{};
                adr.sin_family = AF_INET;
                adr.sin_addr.s_addr = INADDR_ANY;
                adr.sin_port = htons(0);
                bind(sock, (struct sockaddr *)&adr, sizeof(adr));
                memset(&remote_adr, 0, sizeof(remote_adr));
                remote_adr.sin_family = AF_INET; 
                inet_aton(ip, &remote_adr.sin_addr);
                remote_adr.sin_port = htons(PORT);
            }

            int Receive(char *buffer){
                return recv(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0);
            }

            void Send(char* buffer){
                sendto(sock, buffer, MSG_GRVCOPTER::MSG_SIZE, 0, (struct sockaddr *)&remote_adr, size);
            }

            void Ping(){
                sendto(sock, "ping", sizeof("ping"), 0, (struct sockaddr *)&remote_adr, size);
            }


    };

}