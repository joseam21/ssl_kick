#include "./strategy.h"
#include "./main.cpp"

void BasicOffense::robots(){
    int *p = ourRobots();
    for (int n=0; n<6; n++) {
        us[n] = *(p + n);
    } 
};

void BasicOffense::enemyRobots(){
    int *p = theirRobots();
    for (int n=0; n<6; n++) {
        enemy[n] = *(p + n);
    } 
};

void BasicDefense::robots(){
    int *p = ourRobots();
    for (int n=0; n<6; n++) { //TODO: generalize for more than 6 robots
        us[n] = *(p + n);
    } 
}

void BasicDefense::enemyRobots(){
    int *p = theirRobots();
    for (int n=0; n<6; n++) {
        enemy[n] = *(p + n);
    } 
}