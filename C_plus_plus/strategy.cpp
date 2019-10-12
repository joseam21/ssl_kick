#include "./strategy.h"
#include "./main.cpp"

void BasicOffense::setRobots(){
    pose *p = ourRobots();
    pose *q = theirRobots();
    for (int n=0; n<6; n++) { //TODO: generalize for more than 6 robots
        robots[n] = p[n];
    } 
    for (int n=0; n<6; n++) { //TODO: generalize for more than 6 robots
        robots[n + 6] = q[n];
    }
};

bool BasicOffense::clearPath(pose loc1, pose loc2){
    // check for enemy robots along line
    float dist = pow(pow(get<0>(loc1) - get<0>(loc2), 2) + pow(get<1>(loc1) - get<1>(loc2), 2), 0.5);
    float delta_y = get<1>(loc2) - get<1>(loc1);
    float delta_x = get<0>(loc2) - get<0>(loc1);
    for (int n=0; n<6; n++){
        float distFromLine = (delta_y*get<0>(robots[n + 6]) - delta_x*get<1>(robots[n + 6]) + get<0>(loc1)*get<1>(loc2) + get<1>(loc1)*get<0>(loc2)) / dist;
        if (distFromLine < interference_distance)
            return false;
    }
    return true;
}

bool BasicOffense::canScore(){
    pose goalLoc = goal();
    // get location of robot with ball
    pose withBall = robots[posession];
    float dist = pow(get<0>(goalLoc) - get<0>(withBall), 2) + pow(get<1>(goalLoc) - get<1>(withBall), 2);
    if (dist < posession_distance && clearPath(goalLoc, withBall))
        return true;
    return false;
}

int BasicOffense::canPass(){
    for (int n=0; n<6; n++){
        if (n != posession){
            if (clearPath(robots[n], robots[posession]))
                return n;
        }
    }
    return 7;
}

void BasicOffense::play(){
    setRobots();
    if (canScore()) {
        // tell robot to shoot!
    } else{
        int receiver = canPass();
        if (receiver != 7)
            pose receiver_loc = robots[receiver];
            //tell robot to pass!
    }
    // rest of the robots should guard nearest enemy
}
