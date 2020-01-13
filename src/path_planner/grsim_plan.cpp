#include "robotcontrols.h"
#include "RRTX.h"

void setRobotPath(float time) {
  // Print locations of the robots
  // TODO: add these as obstacles
  for (int i=0; i<6; i++) {
    float x = RobotControls::getRobot(true, i).get_x();
    float y = RobotControls::getRobot(true, i).get_y();
    float theta = RobotControls::getRobot(true, i).get_angle();
    std::cout << "Yellow robot id " << i << "(" << x << " " << y << " " << theta << std::endl;
  }

  // TODO: go to waypoints with the following command
  // RobotControls::getRobot(true, i).move_to_location(std::make_pair(x, y);
  // TODO: need to have a flag for when the waypoint is reached
}

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  signal(SIGINT, RobotControls::signalHandler);
  printf("Running\n");
  fflush(stdout);
  RobotControls::go(setRobotPath);
  return 0;
}
