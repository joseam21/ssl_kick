#include "./strategy.h"
#include "./main.cpp"

void BasicOffense::robots(){
    pose *p = ourRobots();
    for (int n=0; n<6; n++) {
        us[n] = p[n];
    } 
};

void BasicOffense::enemyRobots(){
    pose *p = theirRobots();
    for (int n=0; n<6; n++) {
        enemy[n] = p[n];
    } 
};

bool BasicOffense::clearPath(pose loc1, pose loc2){
    // check for enemy robots along line
    float dist = pow(pow(get<0>(loc1) - get<0>(loc2), 2) + pow(get<1>(loc1) - get<1>(loc2), 2), 0.5);
    float delta_y = get<1>(loc2) - get<1>(loc1);
    float delta_x = get<0>(loc2) - get<0>(loc1);
    for (int n=0; n<6; n++){
        float distFromLine = (delta_y*get<0>(enemy[n]) - delta_x*get<1>(enemy[n]) + get<0>(loc1)*get<1>(loc2) + get<1>(loc1)*get<0>(loc2)) / dist;
        if (distFromLine < interference_distance)
            return false;
    }
    return true;
}

bool BasicOffense::canScore(){
    pose goalLoc = goal();
    // get location of robot with ball
    pose withBall = us[posession];
    float dist = pow(get<0>(goalLoc) - get<0>(withBall), 2) + pow(get<1>(goalLoc) - get<1>(withBall), 2);
    if (dist < posession_distance && clearPath(goalLoc, withBall))
        return true;
}

int BasicOffense::canPass(){
    for (int n=0; n<6; n++){
        if (n != posession){
            if (clearPath(us[n], us[posession]))
                return n;
        }
    }
    return 7;
}

