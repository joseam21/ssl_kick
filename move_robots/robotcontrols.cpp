#include "robotcontrols.h"



RobotFSM RobotControls::yellowRobots[] = {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
RobotFSM RobotControls::blueRobots[]= {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
MoveableObject RobotControls::ball(false);
std::chrono::time_point<std::chrono::system_clock> RobotControls::start = std::chrono::system_clock::now();
bool RobotControls::endsignal = false;

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
    return std::make_pair(ball.get_x(),ball.get_y());
}

std::pair<float,float> RobotControls::getCurrentBallSpeed()
{
    return ball.get_vel();
}

void RobotControls::signalHandler(int signum){
    std::string s = "Interrupt Signal: " + std::to_string(signum) + " received\n";
    Log("DEBUG",s);
    endsignal = true;
}

void RobotControls::updateRobotsThread()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    while(true && ! endsignal)
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
                    SSL_DetectionBall dball = detection.balls(0);
                    ball.update_geometry(dball.x()/1000,dball.y()/1000,0,t,dball.confidence());
                }
                int num_blue_robots = detection.robots_blue_size();
                int num_yellow_robots = detection.robots_yellow_size();
                for(int i = 0; i < num_blue_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_blue(i);
                    int id = robot.robot_id();
                    assert(id < 6);
                    assert(id >= 0);
                    blueRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),t,robot.confidence());
                }
                for(int i = 0; i < num_yellow_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_yellow(i);
                    int id = robot.robot_id();
                    assert(id < 6);
                    assert(id >= 0);
                    yellowRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),t,robot.confidence());
                }
            }
            // it's possible for the packet to have geometry information about the field, but for the most part we assume the field is correct and constant, so we have no use for the information
        }
    }
}

void RobotControls::sendRobotCommandThread()
{
    while(true && ! endsignal)
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
    vector<bool> sent(3,false);
    while(true && ! endsignal){
        usleep(1);
        float time = getTime();
        if(time > 1 && !sent[0]){
            sent[0] = true;
            for(int i = 0; i < 6; i++){
                yellowRobots[i].move_to_location(std::make_pair(0,0));
            }
        }else if(time > 4 && !sent[1]){
            sent[1] = true;
            for(int i = 0; i < 6; i++){
                yellowRobots[i].move_to_location(std::make_pair(0,i-2.5));
            }
        }
    }
}

float RobotControls::getTime(){
    auto cur = std::chrono::system_clock::now();
    std::chrono::duration<float> diff = cur - RobotControls::start;
    return diff.count();
}
