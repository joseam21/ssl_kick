#ifndef ROBOTCONTROLS
#define ROBOTCONTROLS

#include <stdio.h>
#include <chrono>
#include <ctime>

#include "robocup_ssl_client.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "robotFSM.h"
#include "network.h"
#include "logger.h"
#include <thread>
#include <utility>
#include <unistd.h>
#include <iostream>

#define TUNING 0

class RobotControls
{

public:
    RobotControls();
    static void go();
    static RobotFSM& getRobot(bool isYellow, int id); // IDs range from 0-5
    static std::pair<float,float> getCurrentBallLoc();
    static std::pair<float,float> getCurrentBallSpeed();
    static void signalHandler(int signum);
private:
    static const int size = 60;// size of the deque for the ball locations, approx 1 second at 60 fps
    
    static RobotFSM yellowRobots[];
    static RobotFSM blueRobots[];
    // TODO: replace with MoveableObject
    static MoveableObject ball;
    static std::chrono::time_point<std::chrono::system_clock> start; // start time of program
    static bool endsignal; // whether the program has been terminated or not
    
    static float getTime(); // returns time in seconds
    
    static void updateRobotsThread();    // Thread to update the Robots from the vision information
    static void sendRobotCommandThread();// Thread to calculate + send commands to the Simulator
    
    static void setRobotStateThread(); // OVERRIDE THIS METHOD TO IMPLEMENT CUSTOM STRATEGY
};


#endif
