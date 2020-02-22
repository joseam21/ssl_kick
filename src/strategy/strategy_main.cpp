#include "robotcontrols.h"
#include "oracle.h"
#include "basic_offense.h"
#include "play.h"
#include <signal.h> 
#include <math.h>

vector<bool> sent(8, false);
void setRobotStateFunction(float time)
{
  Oracle oracle = Oracle();
  if (time > 1 && !sent[0]) {
    std::cout << "Going" << std::endl;
    sent[0] = true;
    Play offense = oracle.choose_play();
  } else if (time > 9 && !sent[2]) {
    std::cout << "Kicking" << std::endl;
    sent[2] = true;
    int robot = oracle.closest_yellow_robot();
    RobotControls::getRobot(true, robot).kick(3, 1, 4);
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

