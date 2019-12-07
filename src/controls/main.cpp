#include "robotcontrols.h"
#include <signal.h> 

vector<bool> sent(3,false);
void setRobotStateFunction(float time)
{
    if(time > 1 && !sent[0]){
        sent[0] = true;
        for(int i = 0; i < 6; i++){
            RobotControls::getRobot(true,i).move_to_location(std::make_pair(0,0));
        }
    }else if(time > 4 && !sent[1]){
        sent[1] = true;
        for(int i = 0; i < 6; i++){
            RobotControls::getRobot(true,i).move_to_location(std::make_pair(0,i-2.5));
        }
    }
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    signal(SIGINT,RobotControls::signalHandler);
    printf("RUNNNING\n");
    fflush(stdout);
    RobotControls::go(setRobotStateFunction);
}

