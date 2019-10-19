#ifndef ROBOTFSM_H
#define ROBOTFSM_H

#include<utility>
#include<deque>
#include<mutex>

#define PI 3.1415926535897
#define V_MAX 1.5
#define V_ANG_MAX 2.4
#define SIZE 60

enum RobotMoveState{
    CONSTANT_DIRECTION=1,   // uses a single float, constant_Direction_dir and tries to go in that direction
    CONSTANT_LOCATION=2,
    VARIABLE_LOCATION_TRACK=3,
    VARIABLE_LOCATION_INTERCEPT=4,
    PAUSE=5
};

enum RobotTurnState{
    CONSTANT_ANGLE=1,
    TRACK_OBJECT=2,
    DIRECTION_OF_MOVEMENT=3
};

class RobotFSM{
public:
    RobotFSM();
    RobotFSM(const RobotFSM& robotFSM);
    RobotFSM(int id1, bool isYellow1);
    void move_In_Direction(float dir);
    void move_To_Location(std::pair<float,float> loc);
    void pause();
    void intercept(std::pair<float,float> * loc);
    void track(std::pair<float,float> * loc);
    void dribble();
    void stop_dribble();
    void pass();
    void kick(float kick_speed_x1, float kick_speed_z1, int kick_tries1);
    void send_Command();
    void set_id(int id1);
    void set_isYellow(bool isYellow1);
    void update_x(float x1);
    void update_y(float y1);
    void update_angle(float angle1);
    float get_x();
    float get_y();
    float get_angle();
    std::pair<float,float> get_speed();
private:
    int id;
    bool isYellow;
    std::deque<float> x, y, angle; // sorted by time, so x[0] is the most recent x variable
    // for angle, 0 is towards the right(yellow goal) side and is the same for both blue and yellow, increases counterclockwise like in radial coordinates, so positive angular velocity turns you left and negative turns you right
    float kick_speed_x, kick_speed_z, kick_tries; // kick_speed_z is chip
    bool spinner;
    RobotMoveState robot_state = RobotMoveState::PAUSE;
    RobotTurnState robot_turn_state = RobotTurnState::CONSTANT_ANGLE;
    float constant_Direction_dir;
    std::pair<float,float> constant_Location_loc;   // first is x, second is y for all location pairs
    std::pair<float,float> * variable_Location_Intercept_loc;
    std::pair<float,float> * variable_Location_Track_loc;
    float constant_Angle_angle = 0;
    std::pair<float,float> * track_Object_loc;
    mutable std::mutex mtx_x, mtx_y, mtx_angle, mtx_robot_state, mtx_robot_turn_state; // used for mutex lock due to threading, lock for x,y,angle, and robot_state
    std::deque<float> error, error1;
    //float errorIntegral, errorIntegral1, error, error1;
};

#endif
