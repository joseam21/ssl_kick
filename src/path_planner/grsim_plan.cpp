#include <math.h>
#include <signal.h>
#include "robotcontrols.h"
#include "RRTX.h"

void moveRobotToPoint(bool isYellow, int robot_id, float target_x, float target_y) {
    bool commandsent = false;
    float threshold = 0.05;

    auto setRobotState = [&] (float time) {
      if (!commandsent) {
        RobotControls::getRobot(isYellow, robot_id).move_to_location(std::make_pair(target_x, target_y));
        commandsent = true;
      }

      if (!RobotControls::getRobot(isYellow, robot_id).has_loc()) {
        return;
      }

      float dist_to_goal_sq = pow(RobotControls::getRobot(isYellow, robot_id).get_x() - target_x, 2) +
                              pow(RobotControls::getRobot(isYellow, robot_id).get_y() - target_y, 2);

      if (pow(dist_to_goal_sq, 0.5) <= threshold) {
        RobotControls::getRobot(isYellow, robot_id).move_pause();
        RobotControls::endsignal = true;
      }
    };

    RobotControls::go(setRobotState);
}

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  signal(SIGINT, RobotControls::signalHandler);
  printf("Running\n");
  fflush(stdout);

  if (argc == 1) {
    moveRobotToPoint(true, 0, 0.0, 0.0);
  } else {
    moveRobotToPoint(strcmp(argv[1], "true") == 0,
                     std::stoi(argv[2]),
                     std::stof(argv[3]), std::stof(argv[4]));
  }
  return 0;
}
