#include <iostream>
#include "../../move_robots/robotcontrols.h"
#include "play.h"
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