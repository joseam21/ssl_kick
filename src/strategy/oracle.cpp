#include "oracle.h"
#include "basic_offense.h"
#include <math.h>
#include <algorithm>

Oracle::Oracle(){
}

int Oracle::find_posession(){
  pair<float, float> ball_loc = RobotControls::getCurrentBallLoc();
  std::vector<float> distances;
  for (int n=0; n < 12; n++) {
    // TODO: Don't hardcode in number of robots
    // Helper function for getting the team of a robot
    RobotFSM& robot = RobotControls::getRobot(n < 6, n % 6);
    float x = robot.get_x();
    float y = robot.get_y();
    float dist = pow(ball_loc.first - x, 2) + pow(ball_loc.second - y, 2);
    distances.push_back(dist);
  }
  auto it = std::min_element(distances.begin(), distances.end());
  float min_dist = *it;

  // Make this threshold a parameter
  if(min_dist < .3){
    return it - distances.begin();
  }
  else{
    return -1;
  }
}

int Oracle::closest_yellow_robot() {
  pair<float, float> ball_loc = RobotControls::getCurrentBallLoc();
  std::vector<float> distances;
  for (int n=0; n < 6; n++) {
    RobotFSM& robot = RobotControls::getRobot(true, n);
    float x = robot.get_x();
    float y = robot.get_y();
    // TODO: Helper function for distance
    float dist = pow(ball_loc.first - x, 2) + pow(ball_loc.second - y, 2);
    distances.push_back(dist);
  }

  auto it = std::min_element(distances.begin(), distances.end());
  return it - distances.begin();
}

Play Oracle::choose_play(){
  int hasball = find_posession();
  //std::cout << "Choosing play"  << hasball << std::endl;
  if (hasball == -1) {
    int closest = closest_yellow_robot();
    std::cout << "Ball not in our posession, sending robot " << closest << " to ball" << std::endl;
    RobotControls::sendRobotToBall(true, closest);
    return BasicOffense(closest);
  } else if(hasball < 6){
    std::cout << "Ball in our posession!" << std::endl;
  	return BasicOffense(hasball);
  }/*
  else{
   return BasicDefense(hasball);
  }*/
}
