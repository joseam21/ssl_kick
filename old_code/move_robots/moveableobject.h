#ifndef MOVEABLE_OBJECT
#define MOVEABLE_OBJECT

#include <utility>
#include <deque>
#include <mutex>
#include <iostream>
#include <assert.h>

const float PI = 3.1415926535897;
const int SIZE = 60;

class MoveableObject{
public:
    MoveableObject(); // by default, the hasOrientation variable is true
    MoveableObject(bool hasOrientation1);
    /** 
     *  Method used to update the information that the Movableobject Object has
     *  If no orientation information is necessary, the angle variable is disregarded
     *  @param floats x, y in m where 0,0 is the center
     *  @param float angle is in radians (angle will be fit to -PI to PI within the function)
     *  @param float time is a float specifying time from start of client
     *  @param float confidence is between 0 and 1
     */
    void update_geometry(float x1, float y1, float angle1, float time1, float confidence1 = 1);
    
    // All "standard" functions that just return data that was last updated
    // returns the last updated x position of the robot in m
    float get_x();
    // returns the last updated y position of the robot in m
    float get_y();
    // returns the last updated angle of the robot in radians from -PI to PI
    float get_angle();
    float get_time();
    
    // returns the last received x and y position as a pair
    std::pair<float,float> get_loc();
    /**
     * Method used to get the velocity of the object
     * returns a pair specifying the x and y velocities
     * Note that the velocity's accuracy is highly dependent on the camera,
     * so doing it over a longer timeframe may be more ideal
     * @param num an integer >= 1 specifying the number of updates to used to calculate
     *     the velocity -- e.g. 1->0th and 1st elements, 5-> 0th and 5th elements 
     * This format is used for the get_ang_vel method as well
     */
    std::pair<float,float> get_vel(int num = 1);
    /** Method used to get the angular velocity of the object in rad/s
     *  @param same as above, num specifies number of updates before
     */
    float get_ang_vel(int num = 1);
    
    // All functions that require processing to get the position based off
    // constant acceleration assumptions
    std::pair<float,float> get_est_loc(float time1, float target_speed = 0, float max_accel = 0.1);
    std::pair<float,float> get_est_vel(float time1, float target_speed = 0, float max_accel = 0.1);
    float get_est_ang_vel(float time1, float target_ang_vel = 0, float max_accel = 0.1);
    float get_est_ang(float time1, float target_ang_vel = 0, float max_accel = 0.1);
protected:
    bool hasOrientation;
    float confidence;
    // sorted by time, so x[0] is the most recent x variable
    // for angle, 0 is towards the right(yellow goal) side and is the same for both blue and yellow, 
    // increases counterclockwise like in radial coordinates, so positive angular velocity turns 
    // you left and negative turns you right
    std::deque<float> x, y, angle, time;
    mutable std::mutex mtx_x, mtx_y, mtx_angle, mtx_time;
};

#endif
