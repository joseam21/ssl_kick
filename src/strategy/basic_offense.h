#ifndef OFFENSE_H
#define OFFENSE_H

#include "play.h"
using namespace std;

class BasicOffense : public Play {
    public: 
        // constructor
        BasicOffense (int hasBall) : Play(hasBall){
            posession = hasBall;
        };
        // execute play
        void play();
};

#endif //OFFENSE_H
