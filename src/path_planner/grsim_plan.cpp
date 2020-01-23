#include <math.h>
#include <signal.h>
#include "robotcontrols.h"
#include "RRTX.h"

void moveRobotToPoint(bool isYellow, int robot_id, float target_x, float target_y) {
    RobotFSM& robot = RobotControls::getRobot(isYellow, robot_id);

    // Initialize RRTX
    Node *goal = new Node(target_x, target_y);
    Node *start = new Node(robot.get_x(), robot.get_y());
    RRTX *tree = new RRTX(-6.0, 6.0, -4.0, 4.0, start, goal);

    // Add other robots as obstacles
    std::vector<Node*> removed = {};
    std::vector<Node*> added;
    for (int i = 0; i < 6; i++) {
        std::cout << "On robot id " << i << std::endl;
        RobotFSM& other_bot = RobotControls::getRobot(!isYellow, i);
        added.push_back(new Node(other_bot.get_x(), other_bot.get_y()));

        if (i != robot_id) {
            RobotFSM& other_bot = RobotControls::getRobot(isYellow, i);
            added.push_back(new Node(other_bot.get_x(), other_bot.get_y()));
        }
    }
    tree->updateObstacles(removed, added);
    std::cout << "Obstacles added." << std::endl;

    // Add nodes to tree
    for (int i = 0; i < 10000; i++) {
        std::cout << i << std::endl;
        tree->step();
    }
    std::cout << "Nodes added." << std::endl;

    float nearest_dist = 0.0;
    Node *parent = NULL;
    std::vector<Node*> nodes;
    tree->V->list(nodes);

    for (Node *node : nodes) {
        if (parent == NULL || start->distance(node) < nearest_dist) {
            if (tree->possiblePath(start, node, tree->O)) {
                parent = node;
                nearest_dist = start->distance(node);
            }
        }
    }

    tree->robot = start;
    tree->robot->parent = parent;
    tree->robot->lmc = parent->lmc + nearest_dist;
    tree->robot->g = parent->g + nearest_dist;

    float last_x, last_y;
    bool command_sent = false;
    float threshold = 0.05;

    auto setRobotState = [&] (float time) {
        // Update goal point
        std::vector<Node*> nextPoints = tree->getNextPoints(1);
        if (nextPoints.size() == 0) {
            RobotControls::endsignal = true;
            return;
        }

        float goal_x, goal_y;
        std::tie(goal_x, goal_y) = nextPoints[0]->getCoords();

        // Check if we need to send command again
        if (command_sent) {
            if (goal_x != last_x) {
                command_sent = false;
            }
            if (goal_y != last_y) {
                command_sent = false;
            }
        }

        // Send command if necessary
        if (!command_sent) {
          robot.move_to_location(std::make_pair(goal_x, goal_y));
          command_sent = true;
          last_x = goal_x;
          last_y = goal_y;
        }


        std::vector<Node*> nextPoints2 = tree->getNextPoints(5);
        float x, y;
        for (int i = 0; i < nextPoints2.size(); i++) {
            std::tie(x, y) = nextPoints2[i]->getCoords();
            std::cout << x << ", " << y << std::endl;
        }
        std::cout << "------------" << std::endl;

        float dist_to_goal_sq = pow(robot.get_x() - goal_x, 2) +
                                pow(robot.get_y() - goal_y, 2);

        if (pow(dist_to_goal_sq, 0.5) <= threshold) {
          robot.move_pause();
          tree->robot = tree->robot->parent;
        }
    };

    std::cout << "Calling go." << std::endl;
    RobotControls::go(setRobotState);
}

void moveRobotToPoints(bool isYellow, int robot_id, std::vector<float> target_xs, std::vector<float> target_ys) {
    RobotFSM& robot = RobotControls::getRobot(isYellow, robot_id);

    bool command_sent = false;
    float threshold = 0.05;
    int current_point = 0;

    auto setRobotState = [&] (float time) {
      if (!command_sent) {
        robot.move_to_location(std::make_pair(target_xs[current_point], target_ys[current_point]));
        command_sent = true;
      }

      if (!robot.has_loc()) {
        return;
      }

      float dist_to_goal_sq = pow(robot.get_x() - target_xs[current_point], 2) +
                              pow(robot.get_y() - target_ys[current_point], 2);

      if (pow(dist_to_goal_sq, 0.5) <= threshold) {
        current_point++;
        if (current_point == target_xs.size()) {
            robot.move_pause();
            RobotControls::endsignal = true;
        } else {
            command_sent = false;
        }
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

  // Quick hack to initialize robots and ids
  auto initRobots = [&] (float time) {
    for (int i = 0; i < 6; i++) {
        if (!RobotControls::getRobot(true, i).has_loc()) {
            return;
        }
        if (!RobotControls::getRobot(false, i).has_loc()) {
            return;
        }
    }

    RobotControls::endsignal = true;
  };
  RobotControls::go(initRobots);
  RobotControls::endsignal = false;

  if (argc == 1) {
    moveRobotToPoint(true, 0, 0.0, 0.0);
  } else {
    moveRobotToPoint(strcmp(argv[1], "true") == 0,
                     std::stoi(argv[2]),
                     std::stof(argv[3]), std::stof(argv[4]));
  }

  return 0;
}
