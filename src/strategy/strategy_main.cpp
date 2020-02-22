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
  int posession = oracle.find_posession();
  if(posession >= 0 && posession < 6 )
    std::cout << RobotControls::getRobot(true,oracle.find_posession()).to_str() << std::endl;
  if (time > 1 && !sent[0]) {
    std::cout << "Going" << std::endl;
    sent[0] = true;
    Play offense = oracle.choose_play();
  } else if (time > 4 && !sent[2]) {
    Play offense = oracle.choose_play();
    sent[2] = true;
    offense.goToGoal();
  } else if (time > 8 && !sent[3]){
    sent[3] = true;
    Play offense = oracle.choose_play();
    if(offense.canScore()){
      offense.kick();
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
