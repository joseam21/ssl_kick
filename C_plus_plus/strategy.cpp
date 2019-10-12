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

