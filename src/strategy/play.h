#ifndef PLAY_H
#define PLAY_H

#include <iostream>
#include "robotcontrols.h"
using namespace std;


class Play {
    public:
        const double posession_distance = 0.2;
        const double interference_distance = 1;
        const std::pair<float, float> goal_loc = std::make_pair(-6, 0);
        int posession;

    public:
        // constructor to set which robot has the ball
        Play (int);
        // check if robot with posession can score
        bool canScore();
        // check if robot with posession can pass
        int canPass();
        // check for path with no enemy robots
        bool clearPath(std::pair<float, float>, std::pair<float, float>);
        // find closest enemy for robot to guard
        int guard(int);
        void goToGoal();
        void kick();
};

#endif //PLAY_H
