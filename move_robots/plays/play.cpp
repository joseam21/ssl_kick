#include "./play.h"

Play::Play(RobotControls controls, int hasBall){
    posession = hasBall;
    controller = controls;
}

bool Play::clearPath(pose loc1, pose loc2){
    // check for enemy robots along line
    float dist = pow(pow(get<0>(loc1) - get<0>(loc2), 2) + pow(get<1>(loc1) - get<1>(loc2), 2), 0.5);
    float delta_y = get<1>(loc2) - get<1>(loc1);
    float delta_x = get<0>(loc2) - get<0>(loc1);
    for (int n=0; n<6; n++){
        RobotFSM interceptor = controller.getRobot(false, n); 
        float distFromLine = (delta_y*interceptor.get_x() - delta_x*interceptor.get_y() + get<0>(loc1)*get<1>(loc2) + get<1>(loc1)*get<0>(loc2)) / dist;
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
    float dist = pow(get<0>(goal_loc) - x, 2) + pow(get<1>(goal_loc) - y, 2);
    pose shooter = std::make_tuple(x, y);
    if (dist < posession_distance && clearPath(goal_loc, shooter))
        return true;
    return false;
}

int Play::canPass(){
    RobotFSM withBall = controller.getRobot(true, posession);
    for (int n=0; n<6; n++){
        if (n != posession){
            RobotFSM receiver = controller.getRobot(true, n);
            if (clearPath(std::make_tuple(receiver.get_x(), receiver.get_y()), std::make_tuple(withBall.get_x(), withBall.get_y())))
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