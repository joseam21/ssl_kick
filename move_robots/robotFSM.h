#ifndef ROBOTFSM_H
#define ROBOTFSM_H

#include<utility>
#include<deque>
#include<mutex>

#define PI 3.1415926535897
#define USE_WHEEL_VEL 0     // toggle whether or not to use wheel velocity 
#define V_MAX 1.5
#define V_ANG_MAX 2.4

#define SIZE 60

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

class RobotFSM{
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
    
    void move_to_intercept(std::pair<float,float> * loc);
    void move_to_track(std::pair<float,float> * loc);
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
    void rotate_to_variable_direction(float * dir);
    void rotate_to_variable_location(std::pair<float,float> * loc);
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
    //TODO: move geometric properties to superclass MovableObject
    /**
     * Updates the robot 
     * @param float specifying the last updated x position, 0 as the center, in m
     * @param float specifying the last updated y position, 0 as the center, in m
     * @param float specifying the last updated orientation/angle 
     * @param float specifying the last updated time of the program in s
     */
    void update_geometry(float x1, float y1, float angle1, float time1);
    // setters
    void set_id(int id1);
    void set_isYellow(bool isYellow1);
    // getters
    // returns the last updated x position of the robot in m
    float get_x();
    // returns the last updated y position of the robot in m
    float get_y();
    // returns the last updated angle of the robot in radians from -PI to PI
    float get_angle();
    // returns the last updated velocity of the robot as a pair of floats in m/s of x and y directions
    std::pair<float,float> get_vel();
    // returns the last updated velocity of the robot as float in rad/s
    float get_ang_vel();
    // returns the speed of the robot in m/s
    float get_speed();
    // methods for estimation of speed and position if telemetry stops
    /** NOT IMPLEMENTED
     * returns a pair of floats specifying the estimation of the current velocity
     * based on acceleration assumptions
     */
    std::pair<float,float> get_vel_est();
    /** NOT IMPLEMENTED
     * returns a pair of floats specifying the estimation of the current position
     * based on acceleration assumptions
     */
    std::pair<float,float> get_pos_est();
private:
    // private data
    int id;
    bool isYellow;
    std::deque<float> x, y, angle, time; // sorted by time, so x[0] is the most recent x variable
    // for angle, 0 is towards the right(yellow goal) side and is the same for both blue and yellow, increases counterclockwise like in radial coordinates, so positive angular velocity turns you left and negative turns you right
    float velangular,velnormal,veltangent;
    //ball manipulation variables
    float kick_speed_x, kick_speed_z, kick_tries; // kick_speed_z is chip
    bool spinner;
    // Robot State variables
    RobotMoveState robot_move_state = MOVE_PAUSE;
    RobotTurnState robot_turn_state = TURN_CONSTANT_DIRECTION;
    // Variables for each robot state
    float constant_direction_dir;
    float * variable_direction_dir;
    std::pair<float,float> constant_location_loc;   // first is x, second is y for all location pairs
    std::pair<float,float> * variable_location_loc;
    // thread stuff
    mutable std::mutex mtx_x, mtx_y, mtx_angle, mtx_time, mtx_robot_move_state, mtx_robot_turn_state; // used for mutex lock due to threading, lock for x,y,angle, and robot_state
    // storing errors for PID alogrithm
    std::deque<float> move_error, angle_error;
    // private functions
    void update_x(float x1);
    void update_y(float y1);
    void update_angle(float angle1);
    void update_time(float time1);
    // resets varaibles so it doesn't interfere with state changes
    void reset_state_variables();
};

#endif
