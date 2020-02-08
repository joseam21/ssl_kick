#ifndef ROBOTFSM_H
#define ROBOTFSM_H

#include<utility>
#include<deque>
#include<mutex>
#include<functional>
#include "moveableobject.h"

#define PI 3.1415926535897
#define USE_WHEEL_VEL 0     // toggle whether or not to use wheel velocity

#define SIZE 60

// NOTE: Robots accelerate at approx 150 rad/s^2 and 5.2 m/s

enum RobotMoveState{
    MOVE_CONSTANT_DIRECTION=1,   // uses constant_direction_dir
    MOVE_CONSTANT_LOCATION=2,
    MOVE_VARIABLE_LOCATION_TRACK=3,
    MOVE_VARIABLE_LOCATION_INTERCEPT=4,
    MOVE_PAUSE=5
};

enum RobotTurnState{
    TURN_CONSTANT_DIRECTION=1,
    TURN_CONSTANT_LOCATION=2,
    TURN_VARIABLE_DIRECTION=3,
    TURN_VARIABLE_LOCATION=4,
    TURN_DIRECTION_OF_MOVEMENT=5
};

class RobotFSM : public MoveableObject{
public:
    RobotFSM();
    RobotFSM(const RobotFSM& robotFSM);
    RobotFSM(int id1, bool isYellow1);
    // 5 directional movement methods
    /**
     * Tells to robot to move in a specific direction indefinitely
     * @param float of angle of movement, in radians from -PI to PI
     */
    void move_in_direction(float dir);
    /**
     * Tells to robot to move to a location indefinitely
     * @param pair of floats that specify location in meters
     */
    void move_to_location(std::pair<float,float> loc);
    /**
     * Tells to robot to pause indefinitely
     */
    void move_pause();

    void move_to_intercept(std::function<std::pair<float,float>(void)> loc_func);
    void move_to_track(std::function<std::pair<float,float>(void)> loc_func);
    // 5 rotational methods
    /**
     * Tells to robot to rotate to a specific direction indefinitely
     * @param float of angle of movement, in radians from -PI to PI
     */
    void rotate_to_direction(float dir);
    /**
     * Tells to robot to rotate to face a specific location indefinitely
     * @param pair of floats specifying position, in radians from -PI to PI
     */
    void rotate_to_location(std::pair<float,float> loc);
    void rotate_to_variable_direction(std::function<float(void)> dir_func);
    void rotate_to_variable_location(std::function<std::pair<float,float>(void)> loc_func);
    /**
     * Tells to robot to rotate to face its direction of movement indefinitely
     * Rotation will probably have higher priority than Movement in Wheel Velocity
     * Calculations (rotate first before moving)
     * @param float of angle of movement, in radians from -PI to PI
     */
    void rotate_to_movement();
    // Ball manipulation methods
    /**
     * Tells to robot to dribble indefinitely (robot by default does not dribble)
     */
    void dribble();
    /**
     * Tells to robot to stop dribbling indefinitely
     */
    void stop_dribble();
    /**
     * Tells the robot to kick for the next ~5 commands (if nothing is passed)
     * @param kick_speed_x1 specifies speed of kick (in m/s?) in direction robot in facing?
     * @param kick_speed_z1 specifies speed of kick (in m/s?) in the air
     * @param kick_tries1 specifies # of commands sent
     */
    void kick(float kick_speed_x1=5, float kick_speed_z1=0, int kick_tries1=5);
    /**
     * Tells the robot to calculate velocities, kicking, dribbling, and send it to the simulator
     * @param float specifying the current time of the program
     */
    void send_Command(float cur_time);
    // setters
    void set_id(int id1);
    void set_isYellow(bool isYellow1);

    //movement variable setters
    void set_max_ang_vel(float new_max_ang_vel);
    void set_max_plane_vel(float new_max_plane_vel);
    void set_max_wheel_vel(float new_max_wheel_vel);
    void reset_max_vel_variables();

    // override base method for updating the geometry, mostly for debug purposes
    void update_geometry(float x1, float y1, float angle1, float time1, float confidence1 = 1);
private:
    // private data
    int id;
    bool isYellow;
    float velangular,velnormal,veltangent;
    // movement setting Variables
    float max_ang_vel = 2;
    float max_plane_vel = 3;
    // TODO: fuse angular and wheel velocity maximums
    float max_wheel_vel = 2;
    //ball manipulation variables
    float kick_speed_x, kick_speed_z, kick_tries; // kick_speed_z is chip
    bool spinner;
    // Robot State variables
    RobotMoveState robot_move_state = MOVE_PAUSE;
    RobotTurnState robot_turn_state = TURN_CONSTANT_DIRECTION;
    // Variables for each robot state
    float constant_direction_dir;
    std::function<float(void)> variable_direction_dir_func;
    std::pair<float,float> constant_location_loc;   // first is x, second is y for all location pairs
    std::function<std::pair<float,float>(void)> variable_location_loc_func;
    // thread stuff
    mutable std::mutex mtx_robot_move_state, mtx_robot_turn_state; // used for mutex lock due to threading, lock for x,y,angle, and robot_state
    // storing errors for PID alogrithm
    std::deque<float> move_error, angle_error;
    // private functions
    // resets varaibles so it doesn't interfere with state changes
    void reset_state_variables();

    std::pair<float,float> compute_plane_vel(float time1);
    float compute_ang_vel(float time1);
};

#endif
