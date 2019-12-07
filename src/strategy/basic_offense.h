#ifndef OFFENSE_H
#define OFFENSE_H

#include "play.h"
using namespace std;

class BasicOffense : public Play {
    public: 
        // constructor
        BasicOffense (RobotControls controls, int hasBall) : Play(controls, hasBall){
            controller = controls;
            posession = hasBall;
        };
        // execute play
        void play();
};

#endif //OFFENSE_H
