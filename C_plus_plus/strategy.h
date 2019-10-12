#include <iostream>
#include <string>
#include <cmath>
using namespace std;

typedef std::tuple<int, int> pose;

class BasicOffense {
    double posession_distance = 0.2;
    double interference_distance = 1;
    int posession; 
    pose ball;
    pose us[];
    pose enemy[];

    public: 
        // constructor to set which robot has the ball
        BasicOffense (int);
        //set our robot locations
        void robots();
        // set enemy robot locations
        void enemyRobots();
        // check if robot with posession can score
        bool canScore();
        // check if robot with posession can pass
        int canPass();
        // check for path with no enemy robots
        bool clearPath(pose, pose);
        // execute play
        void play();
};

BasicOffense::BasicOffense(int possession){
    posession = posession;
}