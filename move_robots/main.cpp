#include <stdio.h>
#include "timer.h"

#include "robocup_ssl_client.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "robotFSM.h"
#include <thread>
#include <utility>
#include <unistd.h>
#include <iostream>

//this is the thread to update the vision information of the board, received from the vision multicast port of the simulator
void updateRobotsThread(RoboCupSSLClient & client, RobotFSM * yellowRobots, RobotFSM * blueRobots, std::pair<float,float> &ballloc)
{
    SSL_WrapperPacket packet;
    while(true)
    {
        if(client.receive(packet))
        {
            if(packet.has_detection())
            {
                SSL_DetectionFrame detection = packet.detection();
                if(detection.balls_size() > 0)
                {
                    SSL_DetectionBall ball = detection.balls(0);
                    ballloc.first = ball.x();
                    ballloc.second = ball.y();
                }
                int num_blue_robots = detection.robots_blue_size();
                int num_yellow_robots = detection.robots_yellow_size();
                for(int i = 0; i < num_blue_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_blue(i);
                    int id = robot.robot_id();
                    blueRobots[id].update_x(robot.x());
                    blueRobots[id].update_y(robot.y());
                    blueRobots[id].update_angle(robot.orientation());
                }
                for(int i = 0; i < num_yellow_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_yellow(i);
                    int id = robot.robot_id();
                    //printf(" %d,",id);
                    //fflush(stdout);
                    yellowRobots[id].update_x(robot.x());
                    yellowRobots[id].update_y(robot.y());
                    yellowRobots[id].update_angle(robot.orientation());
                    //printf("%f\n",robot.orientation());
                }
            }
            // it's possible for the packet to have geometry information about the field, but for the most part we assume the field is correct and constant, so we have no use for the information
        }
    }
}

//this is the thread for each robot to send their action to the simulator depending on their robotState
void sendRobotCommandThread(RobotFSM * yellowRobots, RobotFSM * blueRobots)
{
    while(true)
    {
        usleep(16000);// approx 60 times a second, which is approx how often we get info from vision
        for(int i = 0; i < 6; i++)
        {
            yellowRobots[i].send_Command();
            blueRobots[i].send_Command();
        }
    }
}

//this is the thread to modify the robot states, depending on 
void setRobotStateThread(RobotFSM * yellowRobots, RobotFSM * blueRobots, pair<float,float> & ballloc)
{
    usleep(1000000); // in microseconds
    for(int i = 0; i < 6; i++){
        //yellowRobots[i].move_In_Direction(0);
        yellowRobots[i].move_To_Location(std::make_pair(0,0));
    }
    usleep(5000000); // in microseconds
    for(int i = 0; i < 6; i++){
        yellowRobots[i].move_To_Location(std::make_pair(0,i-2.5));
    }
    while(true){
        std::cout << "Enter team: ";
        std::string ans;
        bool isYellow;
        int id;
        float x,y;
        std::getline(std::cin,ans);
        if(ans[0] == 'b' || ans[0] == 'B'){
            isYellow = false;
        }else if(ans[0] == 'y' || ans[0] == 'Y'){
            isYellow = true;
        }else if(ans[0] == 'e'){
            break;
        }
        std::cout << "Enter ID: ";
        std::cin >> id;
        std::cout << "Enter x: ";
        std::cin >> x; 
        std::cout << "Enter y: ";
        std::cin >> y;
        if(isYellow){
            yellowRobots[id].move_To_Location(std::make_pair(x,y));
        }else{
            blueRobots[id].move_To_Location(std::make_pair(x,y));
        }
        std::getline(std::cin,ans);
            
    }
    usleep(100000000);
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    //printf("Hello\n");
    //fflush(stdout);
    RobotFSM yellowRobots[6];
    RobotFSM blueRobots[6];
    for(int i = 1; i <= 6; i++){
        yellowRobots[i-1].set_id(i-1);
        yellowRobots[i-1].set_isYellow(true);
        blueRobots[i-1].set_id(i-1);
        blueRobots[i-1].set_isYellow(false);
    }
    std::pair<float,float> ballloc;
    RoboCupSSLClient client;
    client.open(true);
    //updateRobotsThread(client,yellowRobots,blueRobots,ballloc);
    std::thread t1(updateRobotsThread,std::ref(client),yellowRobots,blueRobots,std::ref(ballloc));
    std::thread t2(sendRobotCommandThread,yellowRobots,blueRobots);
    std:;thread t3(setRobotStateThread,yellowRobots,blueRobots,std::ref(ballloc));
    t1.join();
    t2.join();
    t3.join();
}

