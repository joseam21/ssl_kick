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

bool sendCommand(bool isYellow, float timestamp, int id, float kickspeedx, float kickspeedz, float veltangent, 
    float velnormal, float velangular, bool spinner, bool wheelsspeed, float wheel1, float wheel2, float wheel3, float wheel4) {
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp(timestamp);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(id);
    command->set_kickspeedx(kickspeedx);
    command->set_kickspeedz(kickspeedz);
    command->set_veltangent(veltangent);
    command->set_velnormal(velnormal);
    command->set_velangular(velangular);
    command->set_spinner(spinner);
    command->set_wheelsspeed(wheelsspeed);
    command->set_wheel1(wheel1);
    command->set_wheel2(wheel2);
    command->set_wheel3(wheel3);
    command->set_wheel4(wheel4);
    //packet.PrintDebugString();
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
        return false;
    }
    close(fd);
    return true;
}

bool sendReplacement(float x, float y, float dir, int id, bool isYellow){
    grSim_Packet packet;
    grSim_RobotReplacement* replacement = packet.mutable_replacement()->add_robots();
    replacement->set_x(x);
    replacement->set_y(y);
    replacement->set_dir(dir);
    replacement->set_id(id);
    replacement->set_yellowteam(isYellow);
    //packet.PrintDebugString();
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
        return false;
    }
    close(fd);
    return true;
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
