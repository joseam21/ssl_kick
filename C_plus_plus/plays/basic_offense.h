#include <iostream>
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