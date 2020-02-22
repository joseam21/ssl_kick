#include "robotcontrols.h"
#include "math.h"
void sampleSetRobotStateFunction(float time){
    if(time > 1){
        RobotControls::getRobot(true,0).move_to_location(std::make_pair(0,0));
    }
}
RobotFSM RobotControls::yellowRobots[] = {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
RobotFSM RobotControls::blueRobots[]= {RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM(),RobotFSM()};
MoveableObject RobotControls::ball(false);
std::chrono::time_point<std::chrono::system_clock> RobotControls::start = std::chrono::system_clock::now();
volatile bool RobotControls::endsignal = false;
std::function<void(float)> RobotControls::setRobotStateFunction = sampleSetRobotStateFunction;

RobotControls::RobotControls(){}

void RobotControls::go(std::function<void(float)> setRobotStateFunction1)
{
    for(int i = 0; i < 6; i++){
        yellowRobots[i].set_id(i);
        yellowRobots[i].set_isYellow(true);
        blueRobots[i].set_id(i);
        blueRobots[i].set_isYellow(false);
    }
    setRobotStateFunction = setRobotStateFunction1;
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
    client.open(false);
    SSL_WrapperPacket packet;
    while(!endsignal)
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
                    if(id >=6 || id < 0){
                      std::cerr << "Invalid ID: PLEASE CHECK THAT SIMULATOR HAS 6 ROBOTS" << std::endl;
                    }
                    blueRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),t,robot.confidence());
                }
                for(int i = 0; i < num_yellow_robots; i++)
                {
                    const SSL_DetectionRobot& robot = detection.robots_yellow(i);
                    int id = robot.robot_id();
                    if(id >=6 || id < 0){
                      std::cerr << "Invalid ID: PLEASE CHECK THAT SIMULATOR HAS 6 ROBOTS" << std::endl;
                    }
                    yellowRobots[id].update_geometry(robot.x()/1000,robot.y()/1000,robot.orientation(),t,robot.confidence());
                }
            }
            // it's possible for the packet to have geometry information about the field, but for the most part we assume the field is correct and constant, so we have no use for the information
        }
    }
}

void RobotControls::sendRobotCommandThread()
{
    while(!endsignal)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
        for(int i = 0; i < 6; i++)
        {
            yellowRobots[i].send_Command(getTime());
            blueRobots[i].send_Command(getTime());
        }
    }
}
void RobotControls::setRobotStateThread()
{
    while(!endsignal){
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        setRobotStateFunction(getTime());
    }
}

float RobotControls::getTime(){
    auto cur = std::chrono::system_clock::now();
    std::chrono::duration<float> diff = cur - RobotControls::start;
    return diff.count();
}

void RobotControls::sendRobotToBall(bool isYellow, int id){
	RobotControls::getRobot(isYellow,id).dribble();
	RobotControls::getRobot(isYellow,id).rotate_to_variable_location([&](){return RobotControls::getCurrentBallLoc();});
	RobotControls::getRobot(isYellow,id).move_to_track([&, isYellow, id](){
		std::pair<float,float> ball_loc = RobotControls::getCurrentBallLoc();
		RobotFSM& robot = getRobot(isYellow,id);
		std::pair<float,float> robot_loc = getRobot(isYellow,id).get_loc();
		float angle = atan2(robot_loc.second-ball_loc.second,robot_loc.first-ball_loc.first);
		std::pair<float,float> ideal_loc = {ball_loc.first+.085 *cos(angle),ball_loc.second+.085*sin(angle)};
		return ideal_loc;
		}
	);

}
