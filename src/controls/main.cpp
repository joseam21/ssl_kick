#include "robotcontrols.h"
#include <signal.h> 

vector<bool> sent(4,false);
void setRobotStateFunction(float time)
{
	if(time > 0 && !sent[2]){
		sent[2] = true;
		for(int i = 0; i < 6; i++){
			RobotControls::getRobot(false,i).move_to_location({-4,i-2.5});
		}
	}
    if(time > 0 && !sent[0]){
        sent[0] = true;
        for(int i = 0; i < 6; i++){
            RobotControls::getRobot(true,i).move_to_location(std::make_pair(-3,i-2.5));
        }
    }else if(time > 5 && !sent[1]){
        sent[1] = true;
        for(int i = 0; i < 6; i++){
            RobotControls::getRobot(true,i).move_to_location(std::make_pair(-2+i,i-2.5));
        }
    }else if(time > 8 && !sent[3]){
		sent[3] = true;
		printf("WOW\n");
		fflush(stdout);
		RobotControls::sendRobotToBall(true,1);
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

