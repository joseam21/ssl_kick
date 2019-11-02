#include "robotcontrols.h"



RobotFSM RobotControls::yellowRobots[] = {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
RobotFSM RobotControls::blueRobots[]= {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
deque<std::pair<float,float>> RobotControls::ballloc = deque<std::pair<float,float>>();
deque<float> RobotControls::balltime = deque<float>();
std::chrono::time_point<std::chrono::system_clock> RobotControls::start = std::chrono::system_clock::now();

RobotControls::RobotControls()
{
    // setup the robots   
    
    for(int i = 0; i < 6; i++){
        yellowRobots[i].set_id(i);
        yellowRobots[i].set_isYellow(true);
        blueRobots[i].set_id(i);
        blueRobots[i].set_isYellow(false);
    }
}

void RobotControls::go()
{
    std::thread t1(updateRobotsThread);
    std::thread t2(sendRobotCommandThread);
    std::thread t3(setRobotStateThread);
    t1.join();
    t2.join();
    t3.join();
}

RobotFSM& RobotControls::getRobot(bool isYellow, int id)
{
    if(isYellow)
    {
        return yellowRobots[id];
    }
    else{
        return blueRobots[id];
    }
}

std::pair<float,float> RobotControls::getCurrentBallLoc()
{
    return ballloc.front();
}

std::pair<float,float> RobotControls::getCurrentBallSpeed()
{
    return std::make_pair((ballloc[0].first-ballloc[1].first)/(balltime[0]-balltime[1]),
            (ballloc[0].second-ballloc[1].second)/(balltime[0]-balltime[1]));
}

void RobotControls::updateRobotsThread()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    while(true)
    {
        if(client.receive(packet))
        {
            if(packet.has_detection())
            {
                float t = getTime();
                //printf("Packet Received at Time: %f\n", t);
                //fflush(stdout);
                SSL_DetectionFrame detection = packet.detection();
                if(detection.balls_size() > 0)
                {
                    SSL_DetectionBall ball = detection.balls(0);
                    ballloc.push_front(std::make_pair(ball.x(),ball.y()));
                }
                int num_blue_robots = detection.robots_blue_size();
                int num_yellow_robots = detection.robots_yellow_size();
                for(int i = 0; i < num_blue_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_blue(i);
                    int id = robot.robot_id();
                    assert(id < 6);
                    assert(id >= 0);
                    blueRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),robot.confidence(),t);
                }
                for(int i = 0; i < num_yellow_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_yellow(i);
                    int id = robot.robot_id();
                    assert(id < 6);
                    assert(id >= 0);
                    yellowRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),robot.confidence(),t);
                }
            }
            // it's possible for the packet to have geometry information about the field, but for the most part we assume the field is correct and constant, so we have no use for the information
        }
    }
}

void RobotControls::sendRobotCommandThread()
{
    while(true)
    {
        usleep(16000);// approx 60 times a second, which is approx how often we get info from vision
        for(int i = 0; i < 6; i++)
        {
            float t1 = (float)(getTime());
            yellowRobots[i].send_Command(t1);
            float t2 = (float)(getTime());
            blueRobots[i].send_Command(t2);
        }
    }
}

void RobotControls::setRobotStateThread()
{
    //printf("WOWWWWWW\n");
    //fflush(stdout);
    usleep(1000000); // in microseconds
    for(int i = 0; i < 6; i++){
        //yellowRobots[i].move_in_direction(0);
        yellowRobots[i].move_to_location(std::make_pair(0,0));
    }
    //printf("WOWWWWWW\n");
    //fflush(stdout);
    usleep(5000000); // in microseconds
    for(int i = 0; i < 6; i++){
        yellowRobots[i].move_to_location(std::make_pair(0,i-2.5));
    }
    usleep(100000000);
}

float RobotControls::getTime(){
    auto cur = std::chrono::system_clock::now();
    std::chrono::duration<float> diff = cur - RobotControls::start;
    return diff.count();
}
