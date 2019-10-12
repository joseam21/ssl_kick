#include <iostream>
#include <string>
#include <cmath>
using namespace std;

typedef std::tuple<int, int> pose;

class BasicOffense {
    double posession_distance = 0.2;
    pose ball;
    pose us[];
    pose enemy[];

    public: 
        //set our robot locations
        void robots();
        // set enemy robot locations
        void enemyRobots();
        // check if robot with posession can score
        bool canScore();
        // check if robot with posession can pass
        int canPass();
};