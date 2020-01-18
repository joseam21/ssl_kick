#include "robotcontrols.h"
#include "oracle.h"
#include "basic_offense.h"
#include "play.h"
#include <signal.h> 

vector<bool> sent(3,false);
void setRobotStateFunction(float time)
{
  Oracle oracle = Oracle();
  printf("Possession: %d\n", oracle.find_posession());
  Play offense = oracle.choose_play();
  printf("Can score %d\n", offense.canScore());
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

