#ifndef ORACLE_H
#define ORACLE_H

// #include "../robotcontrols.h"
#include "basic_offense.h"
using namespace std;

class Oracle{
    public:
        RobotControls controller;
        
        //constructor
        Oracle (RobotControls);
        // determine who is in posession of the ball
        int find_posession();
        // choose what play to perform
        Play choose_play();
};

Oracle::Oracle(RobotControls controls){
    controller = controls;
}

#endif // ORACLE_H