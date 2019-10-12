#include <iostream>
#include <string>
#include <cmath>
using namespace std;

typedef std::tuple<int, int> pose;

pose ball(){
    // returns the coordinates of the ball
    return make_tuple(1, 1); //placeholder for cmake
};

pose* ourRobots(){
    // returns a pointer to an array of coordinates of our robots
    // pointer because apparently you can't return arrays in C++
    static pose arr[6]; //placeholder for cmake
    return arr;
};

pose* theirRobots(){
    // same as ourRobots but for the other robots
    static pose arr[6]; //placeholder for cmake
    return arr;
};

pose goal(){
    // return the coordinates of the goal
    return make_tuple(1, 1); //placeholder for cmake
};

int main(){
    return 0;
}