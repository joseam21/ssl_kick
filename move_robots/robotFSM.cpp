#include "robotFSM.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "network.h"
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>


float get_angle_diff(float angle1, float angle2);
float get_PID_result(float new_error, std::deque<float> &time, std::deque<float> &error, float K_p, float K_i, float K_d, float min_result, float max_result);
void wheel_velocities(float velnormal, float veltangent, float velangular, float *wheels);

RobotFSM::RobotFSM()
{
    id = -1;
    robot_move_state = MOVE_PAUSE;
    kick_tries = 0;
    spinner = false;
}

RobotFSM::RobotFSM(const RobotFSM& robotFSM)
{
    id = -1;
    robot_move_state = MOVE_PAUSE;
    kick_tries = 0;
    spinner = false;
}

RobotFSM::RobotFSM(int id1, bool isYellow1) : id(id1),isYellow(isYellow1)
{
    robot_move_state = MOVE_PAUSE;
    kick_tries = 0;
    spinner = false;
}

void RobotFSM::move_in_direction(float dir)
{
    mtx_robot_move_state.lock();
    robot_move_state = MOVE_CONSTANT_DIRECTION;
    reset_state_variables();
    constant_direction_dir = dir;
    mtx_robot_move_state.unlock();
}

void RobotFSM::move_to_location(std::pair<float,float> loc)
{
    mtx_robot_move_state.lock();
    robot_move_state = MOVE_CONSTANT_LOCATION;
    reset_state_variables();
    constant_location_loc = std::pair<float,float>(loc);
    mtx_robot_move_state.unlock();
}

void RobotFSM::move_pause()
{
    mtx_robot_move_state.lock();
    robot_move_state = MOVE_PAUSE;
    reset_state_variables();
    mtx_robot_move_state.unlock();
}

void RobotFSM::move_to_intercept(std::pair<float,float> * loc)
{
    mtx_robot_move_state.lock();
    robot_move_state = MOVE_VARIABLE_LOCATION_INTERCEPT;
    reset_state_variables();
    variable_location_loc = loc;
    mtx_robot_move_state.unlock();
}

void RobotFSM::move_to_track(std::pair<float,float> * loc)
{
    mtx_robot_move_state.lock();
    robot_move_state = MOVE_VARIABLE_LOCATION_TRACK;
    reset_state_variables();
    variable_location_loc = loc;
    mtx_robot_move_state.unlock();
}

void RobotFSM::rotate_to_direction(float dir)
{
    mtx_robot_turn_state.lock();
    robot_turn_state = TURN_CONSTANT_DIRECTION;
    reset_state_variables();
    constant_direction_dir = dir;
    mtx_robot_turn_state.unlock();
}

void RobotFSM::rotate_to_location(std::pair<float,float> loc)
{
    mtx_robot_turn_state.lock();
    robot_turn_state = TURN_CONSTANT_LOCATION;
    reset_state_variables();
    constant_location_loc = loc;
    mtx_robot_turn_state.unlock();
}

void RobotFSM::rotate_to_variable_direction(float * dir)
{
    mtx_robot_turn_state.lock();
    robot_turn_state = TURN_VARIABLE_DIRECTION;
    reset_state_variables();
    variable_direction_dir = dir;
    mtx_robot_turn_state.unlock();
}

void RobotFSM::rotate_to_variable_location(std::pair<float,float> * loc)
{
    mtx_robot_turn_state.lock();
    robot_turn_state = TURN_VARIABLE_LOCATION;
    reset_state_variables();
    variable_location_loc = loc;
    mtx_robot_turn_state.unlock();
}

void RobotFSM::rotate_to_movement()
{
    mtx_robot_turn_state.lock();
    robot_turn_state = TURN_DIRECTION_OF_MOVEMENT;
    reset_state_variables();
    mtx_robot_turn_state.unlock();
}

void RobotFSM::dribble()
{
    spinner = true;
}

void RobotFSM::stop_dribble()
{
    spinner = false;
}

void RobotFSM::kick(float kick_speed_x1, float kick_speed_z1, int kick_tries1)
{
    kick_speed_x = kick_speed_x1;
    kick_speed_z = kick_speed_z1;
    kick_tries = kick_tries1;
}

void RobotFSM::send_Command(float cur_time)
{
    bool wheelsspeed1;
    float kick_speed_x1, kick_speed_z1, wheel1,wheel2,wheel3,wheel4;
    wheelsspeed1 = USE_WHEEL_VEL == 1;
    if(kick_tries > 0)
    {
        kick_speed_x1 = kick_speed_x;
        kick_speed_z1 = kick_speed_z;
        kick_tries--;
    }
    else {
        kick_speed_x1 = 0;
        kick_speed_z1 = 0;
    }
    std::pair<float,float> vels = compute_plane_vel(cur_time);
    veltangent = vels.first;
    velnormal = vels.second;
    velangular = compute_ang_vel(cur_time);
    if(USE_WHEEL_VEL){
        float * wheels = new float[4]; 
	    wheel_velocities(velnormal,veltangent,velangular, wheels);
        wheel1 = *wheels;
        wheel2 = *(wheels+1);
        wheel3 = *(wheels+2);
        wheel4 = *(wheels+3);
	delete wheels;
    }else
    {
        wheel1 = 0;
        wheel2 = 0;
        wheel3 = 0;
        wheel4 = 0;
    }
    sendCommand(isYellow,cur_time,id,kick_speed_x1,kick_speed_z1,veltangent,velnormal,
        velangular,spinner,wheelsspeed1,wheel1,wheel2,wheel3,wheel4);
}

void RobotFSM::set_id(int id1)
{
    id = id1;
}
void RobotFSM::set_isYellow(bool isYellow1)
{
    isYellow = isYellow1;
}

void RobotFSM::reset_state_variables()
{
    move_error.clear();
    angle_error.clear();
}

std::pair<float,float> RobotFSM::compute_plane_vel(float time1)
{
    mtx_robot_move_state.lock();
    mtx_time.lock();
    std::pair<float,float> res;
    switch(robot_move_state)
    {
        case MOVE_CONSTANT_DIRECTION:
        {
            float angle_diff = get_angle_diff(get_angle(), constant_direction_dir);
            res = std::make_pair(cos(angle_diff*-1)*V_MAX,sin(angle_diff*-1)*V_MAX);
            break;
        }
        case MOVE_CONSTANT_LOCATION:
        {
            float xdif = constant_location_loc.first - get_x();
            float ydif = constant_location_loc.second - get_y();
            float angle1 = atan2(ydif,xdif);
            float angle_diff = get_angle_diff(get_angle(), angle1);
            float new_error = sqrt(pow(xdif,2)+pow(ydif,2));
            // I'm wondering if a cube root function on the error calclulation would improve PID
            //printf("Y: %d, ID: %d ,angle: %f\n",isYellow?0:1,id,angle_diff);
            //fflush(stdout);
            const float K_p = 3.0;
            const float K_i = 0.06;
            const float K_d = 0.02;
            float optimal_velocity = get_PID_result(new_error, time, move_error,K_p,K_i,K_d,-V_MAX,V_MAX);
            res = std::make_pair(cos(angle_diff*-1)*optimal_velocity,sin(angle_diff*-1)*optimal_velocity);
            break;
        }
        case MOVE_PAUSE:
        {
            res= std::make_pair(0,0);
            break;
        }
        case MOVE_VARIABLE_LOCATION_INTERCEPT:
        {
            res= std::make_pair(0,0);
            break;
        }
        case MOVE_VARIABLE_LOCATION_TRACK:
        {
            res= std::make_pair(0,0);
            break;
        }
    
    }
    mtx_time.unlock();
    mtx_robot_move_state.unlock();
    return res;
}

float RobotFSM::compute_ang_vel(float time1)
{
    mtx_robot_turn_state.lock();
    mtx_time.lock();
    float res;
    switch(robot_turn_state)
    {
        case TURN_CONSTANT_DIRECTION:
        {
            float new_error = get_angle_diff(get_angle(),constant_direction_dir);
            const float K_p = -3.5;
            const float K_i = -0.06;
            const float K_d = -0.001;
            res = get_PID_result(new_error,time,angle_error, K_p,K_i,K_d,-V_ANG_MAX,V_ANG_MAX);
            //res = 0;
            break;
        }
        case TURN_CONSTANT_LOCATION:
        {
            res = 0;
            break;
        }
        case TURN_VARIABLE_DIRECTION:
        {
            res = 0;
            break;
        }
        case TURN_VARIABLE_LOCATION:
        {
            res = 0;
            break;
        }
        case TURN_DIRECTION_OF_MOVEMENT:
        {
            res = 0;
            break;
        }
    }
    mtx_time.unlock();
    mtx_robot_turn_state.unlock();
    return res;
}

// Helper Functions below

float get_angle_diff(float angle1, float angle2)
{
    float diff = angle1 - angle2 + 4*PI;
    while(diff > PI)
    {
        diff -= 2*PI;
    }
    return diff;
}


void wheel_velocities(float velnormal, float veltangent, float velangular, float * wheels){
	// velnormal  is positive to the right of the robot 
	// veltangent is positvite straight forward
	// velangular is the angular velocity in rads/s (positive counter clockwise)

	// NB! The following three variables are the ones used the simulator
	/*
	double phi_1 = 33*M_PI/180;
	double phi_2 = 45*M_PI/180;
	double R = 0.0289;
	*/
	double scaling_factor = 3.5;
	velnormal = -velnormal;
	
	//* This can also be correct
	double phi_1 = 30*M_PI/180;
	double phi_2 = 45*M_PI/180;
	double R = 0.027;
	//*/	
	// Find out if the following code can be vectorized later
	//* C
	// Caused weird behaviour, probably different implementation in simulator
    wheels[3] = (-sin(phi_1)*velnormal + cos(phi_1)*veltangent + R*velangular*scaling_factor)/R;
    wheels[0] = (-sin(phi_1)*velnormal - cos(phi_1)*veltangent + R*velangular*scaling_factor)/R;
    wheels[1] = (sin(phi_2)*velnormal - cos(phi_2)*veltangent + R*velangular*scaling_factor)/R;
    wheels[2] = (sin(phi_2)*velnormal + cos(phi_2)*veltangent + R*velangular*scaling_factor)/R;
	//*/
	// We ARE SENDING ANGULAR VELOCITIES TO THE WHEELS
    return;
}



float get_PID_result(float new_error, std::deque<float> &time, std::deque<float> &error, 
          float K_p, float K_i, float K_d, float min_result = -99, float max_result = 99)
{
    float error_integral = 0;
    float error_derivative = 0;
    if(error.size() > 1 && time.size() > 1)
    {
        error_derivative = (new_error - error.front())/(time[0]-time[1]);
        /*if(error_derivative < -0.5){
            printf("New: %f    Old: %f    NewT: %f    OldT: %f\n",new_error,error.front(),time[0],time[1]);
            fflush(stdout);
        }*/
        for(int i = 0; i < error.size(); i++)
        {
            error_integral += error[i];
        }
    }
    if(!isnormal(error_derivative))
    {
        error_derivative = (new_error-error.front()*60);
    }
    error.push_front(new_error);
    while(error.size() > SIZE)
    {
        error.pop_back();
    }
    float result = K_p*new_error+K_i*error_integral+K_d*error_derivative;
    /*if(result < -0.5){
        printf("P: %f,  I: %f,  D:  %f,  res: %f  \n",K_p*new_error,K_i*error_integral,
        K_d*error_derivative,result);
    }*/
    if(result > max_result)
    {
        result = max_result;
    }
    if(result < min_result)
    {
        result = min_result;
    }
    return result;
}
