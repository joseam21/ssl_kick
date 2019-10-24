#include "robotFSM.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>


float get_angle_diff(float angle1, float angle2);
float get_PID_result(float new_error, std::deque<float> &time, std::deque<float> &error, float K_p, float K_i, float K_d, float min_result, float max_result);

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

void RobotFSM::kick(float kick_speed_x1 = 5, float kick_speed_z1 = 0, int kick_tries1 = 5)
{
    kick_speed_x = kick_speed_x1;
    kick_speed_z = kick_speed_z1;
    kick_tries = kick_tries1;
}

void RobotFSM::send_Command(float cur_time)
{
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(id);
    command->set_wheelsspeed(false);
    command->set_spinner(false);
    if(kick_tries > 0)
    {
        command->set_kickspeedx(kick_speed_x);
        command->set_kickspeedz(kick_speed_z);
        kick_tries--;
    }
    else {
        command->set_kickspeedx(0);
        command->set_kickspeedz(0);
    }
    veltangent = 0;
    velnormal = 0;
    velangular =0;
    mtx_robot_move_state.lock();
    switch(robot_move_state)
    {
        case MOVE_CONSTANT_DIRECTION:
        {
            float angle_diff = get_angle_diff(get_angle(), constant_direction_dir);
            veltangent = cos(angle_diff*-1)*V_MAX;
            velnormal = sin(angle_diff*-1)*V_MAX;
            break;
        }
        case MOVE_CONSTANT_LOCATION:
        {
            float xdif = constant_location_loc.first - get_x();
            float ydif = constant_location_loc.second - get_y();
            float angle1 = atan2(ydif,xdif);
            float angle_diff = get_angle_diff(get_angle(), angle1);
            float new_error = sqrt(pow(xdif,2)+pow(ydif,2));
            const float K_p = 1.0;
            const float K_i = 0.00006;
            const float K_d = 0.00002;
            float optimal_velocity = get_PID_result(new_error, time, move_error,K_p,K_i,K_d,-V_MAX,V_MAX);
            veltangent = cos(-1*angle_diff)*optimal_velocity;
            velnormal = sin(-1*angle_diff)*optimal_velocity;
            break;
        }
        case MOVE_PAUSE:
        {
            veltangent = 0;
            velnormal = 0;
            break;
        }
        case MOVE_VARIABLE_LOCATION_INTERCEPT:
        {
            veltangent = 0;
            velnormal = 0;
            break;
        }
        case MOVE_VARIABLE_LOCATION_TRACK:
        {
            veltangent = 0;
            velnormal = 0;
            break;
        }
    
    }
    mtx_robot_move_state.unlock();
    mtx_robot_turn_state.lock();
    switch(robot_turn_state)
    {
        case TURN_CONSTANT_DIRECTION:
        {
            float new_error = get_angle_diff(get_angle(),constant_direction_dir);
            const float K_p = -1.3;
            const float K_i = -0.06;
            const float K_d = -0.02;
            velangular = get_PID_result(new_error,time,angle_error, K_p,K_i,K_d,-V_ANG_MAX,V_ANG_MAX);
            break;
        }
        case TURN_CONSTANT_LOCATION:
        {
            velangular = 0;
            break;
        }
        case TURN_VARIABLE_DIRECTION:
        {
            velangular = 0;
            break;
        }
        case TURN_VARIABLE_LOCATION:
        {
            velangular = 0;
            break;
        }
        case TURN_DIRECTION_OF_MOVEMENT:
        {
            velangular = 0;
            break;
        }
    }
    mtx_robot_turn_state.unlock();
    command->set_veltangent(veltangent);
    command->set_velnormal(velnormal);
    command->set_velangular(velangular);
    
    if(id == 0 && isYellow)
    {
        packet.PrintDebugString();
    }
    // Serialize
    std::string packet_str;
    packet.SerializeToString(&packet_str);

    // Send protobuf message
    sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        perror("cannot open socket");
        return;
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(20011);
    if (sendto(fd, packet_str.c_str(), packet_str.length()+1, 0, // +1 to include terminator
               (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("cannot send message");
        close(fd);
        return;
    }
    close(fd);
    
    //printf("\nDONE\n");
}

void RobotFSM::update_geometry(float x1, float y1, float angle1, float time1)
{
    // std::pair<float,float> pos_est;
    mtx_x.lock();
    mtx_y.lock();
    mtx_angle.lock();
    mtx_time.lock();
    update_x(x1);
    update_y(y1);
    update_angle(angle1);
    update_time(time1);
    mtx_x.unlock();
    mtx_y.unlock();
    mtx_angle.unlock();
    mtx_time.unlock();
    // printf("Error in X: %f,     Error in Y: %f \n",get_x()-pos_est.first,get_y()-pos_est.second); 
}

void RobotFSM::update_x(float x1)
{
    x.push_front(x1/1000);
    while(x.size() > SIZE)
    {
        x.pop_back();
    }
}
void RobotFSM::update_y(float y1)
{
    y.push_front(y1/1000);
    while(y.size() > SIZE)
    {
        y.pop_back();
    }
}
void RobotFSM::update_angle(float angle1)
{
    angle.push_front(get_angle_diff(angle1,0));
    while(angle.size() > SIZE)
    {
        angle.pop_back();
    }
}

void RobotFSM::update_time(float time1)
{
    time.push_front(time1);
    while(time.size() > SIZE)
    {
        time.pop_back();
    }
}

void RobotFSM::set_id(int id1)
{
    id = id1;
}
void RobotFSM::set_isYellow(bool isYellow1)
{
    isYellow = isYellow1;
}


float RobotFSM::get_x()
{
    if(!x.empty())
    {
        return x[0];
    }
    else{
        printf("ERROR: Accessing position while position info is empty");
        return 0;
    }
}

float RobotFSM::get_y()
{
    if(!y.empty())
    {
        return y[0];
    }
    else{
        printf("ERROR: Accessing position while position info is empty");
        return 0;
    }
}
float RobotFSM::get_angle()
{
    if(!angle.empty())
    {
        return angle[0];
    }
    else{
        printf("ERROR: Accessing position while position info is empty");
        return 0;
    }
}

std::pair<float,float> RobotFSM::get_vel()
{
    mtx_x.lock();
    mtx_y.lock();
    if(x.size() > 1 && y.size() > 1)
    {
        return std::make_pair((x[0]-x[1])/(time[0]-time[1]),(y[0]-y[1])/(time[0]-time[1]));
    }
    else{
        printf("ERROR: Accessing position while position info is empty");
        return std::make_pair(0.0,0.0);
    }
    mtx_x.unlock();
    mtx_y.unlock();
}

float RobotFSM::get_speed()
{
    std::pair<float,float> vel = get_vel();
    return sqrt(pow(vel.first,2)+pow(vel.second,2));
}

std::pair<float,float> RobotFSM::get_vel_est()
{
    
}

std::pair<float,float> RobotFSM::get_pos_est()
{
    
}

void RobotFSM::reset_state_variables()
{
    move_error.clear();
    angle_error.clear();
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

float get_PID_result(float new_error, std::deque<float> &time, std::deque<float> &error, float K_p, float K_i, float K_d, float min_result = -99, float max_result = 99)
{
    float error_integral = 0;
    float error_derivative = 0;
    if(error.size() > 1 && time.size() > 1)
    {
        error_derivative = (new_error - error.front())/(time[0]-time[1]);
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
