#include "./play.h"

Play::Play(RobotControls controls, int hasBall){
    posession = hasBall;
    controller = controls;
}

bool Play::clearPath(std::pair<float, float> loc1, std::pair<float, float> loc2){
    // check for enemy robots along line
    float dist = pow(pow(loc1.first - loc2.first, 2) + pow(loc1.second - loc2.second, 2), 0.5);
    float delta_y = loc2.second - loc1.second;
    float delta_x = loc2.first - loc1.first;
    for (int n=0; n<6; n++){
        RobotFSM interceptor = controller.getRobot(false, n); 
        float distFromLine = (delta_y*interceptor.get_x() - delta_x*interceptor.get_y() + loc1.first*loc2.second + loc1.second*loc2.first) / dist;
        if (distFromLine < interference_distance)
            return false;
    }
    return true;
}

bool Play::canScore(){
    // get robot with ball
    RobotFSM withBall = controller.getRobot(true, posession);
    float x = withBall.get_x();
    float y = withBall.get_y();
    float dist = pow(goal_loc.first - x, 2) + pow(goal_loc.second - y, 2);
    std::pair<float, float> shooter = std::make_pair(x, y);
    if (dist < posession_distance && clearPath(goal_loc, shooter))
        return true;
    return false;
}

int Play::canPass(){
    RobotFSM withBall = controller.getRobot(true, posession);
    for (int n=0; n<6; n++){
        if (n != posession){
            RobotFSM receiver = controller.getRobot(true, n);
            if (clearPath(std::make_pair(receiver.get_x(), receiver.get_y()), std::make_pair(withBall.get_x(), withBall.get_y())))
                return n;
        }
    }
    return 7;
}

int Play::guard(int robo){
    float min_dist = 1000000;
    int nearest;
    RobotFSM robot = controller.getRobot(true, robo);
    float x = robot.get_x();
    float y = robot.get_y();
    for (int n=0; n<6; n++){
        RobotFSM enemy = controller.getRobot(false, n);
        float enemy_x = enemy.get_x();
        float enemy_y = enemy.get_y();
        float dist = pow(enemy_x - x, 2) + pow(enemy_y - y, 2);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = n;
        }
    }
    return nearest;
}