#include "network.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

bool sendVelocity(int robot_id, double veltangent, double velnormal, double velangular) {
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(true);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(robot_id);
    command->set_veltangent(veltangent);
    command->set_velnormal(velnormal);
    command->set_velangular(velangular);
    command->set_wheelsspeed(false);
    command->set_spinner(false);
    command->set_kickspeedx(1.0);
    command->set_kickspeedz(1.0);
    packet.PrintDebugString();

    // Serialize
    std::string packet_str;
    packet.SerializeToString(&packet_str);

    // Send protobuf message
    sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        perror("cannot open socket");
        return false;
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(20011);
    if (sendto(fd, packet_str.c_str(), packet_str.length()+1, 0, // +1 to include terminator
               (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("cannot send message");
        close(fd);
        return -1;
    }
    close(fd);
    printf("\nDONE\n");
}

void getLocation(int robot_id) {
    std::cout << "HII" << std::endl;
    char * in_buffer = new char[65536];

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("cannot open socket");
    }
    sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("224.5.23.2");
    servaddr.sin_port = htons(20011);

    SSL_WrapperPacket packet;
    if (recvfrom(fd, in_buffer, 65536, 0, (sockaddr*)&servaddr, (socklen_t*)sizeof(servaddr)) < 0) {
        std::cout << "HISSDFSDFSDI" << std::endl;
        perror("cannot receive message");
        close(fd);
        printf("\nDONE\n");
    }
    std::cout << "AAAA" << std::endl;
    close(fd);
    printf("\nDONE\n");

    return;
}
