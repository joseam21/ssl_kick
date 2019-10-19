#include <iostream>
using namespace std;

typedef std::tuple<int, int> pose;

class Play {
    public:
        const double posession_distance = 0.2;
        const double interference_distance = 1;
        int posession; 
        pose ball;
        pose robots[];

    public: 
        // constructor to set which robot has the ball
        Play (int);
        //set robot locations, first 6 should be ours, next 6 enemy
        void setRobots();
        // check if robot with posession can score
        bool canScore();
        // check if robot with posession can pass
        int canPass();
        // check for path with no enemy robots
        bool clearPath(pose, pose);
        // find closest enemy for robot to guard
        int guard(int);
};

Play::Play(int hasBall){
    posession = hasBall;
}