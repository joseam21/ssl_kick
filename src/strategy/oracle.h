#ifndef ORACLE_H
#define ORACLE_H

class Play;

// #include "basic_offense.h"
#include "robotcontrols.h"
using namespace std;

class Oracle{
    public:
        //constructor
        Oracle ();
        // determine who is in posession of the ball
        int find_posession();
        // choose what play to perform
        Play choose_play();
};

#endif // ORACLE_H
