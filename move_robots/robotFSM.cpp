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

RobotFSM::RobotFSM()
{
    id = -1;
    robot_state = RobotMoveState::PAUSE;
    kick_tries = 0;
    spinner = false;
}

RobotFSM::RobotFSM(int id1, bool isYellow1) : id(id1),isYellow(isYellow1)
{
    robot_state = RobotMoveState::PAUSE;
    kick_tries = 0;
    spinner = false;
}

void RobotFSM::move_In_Direction(float dir)
{
    mtx_robot_state.lock();
    robot_state = RobotMoveState::CONSTANT_DIRECTION;
    error.clear();
    constant_Direction_dir = dir;
    mtx_robot_state.unlock();
}

void RobotFSM::move_To_Location(std::pair<float,float> loc)
{
    mtx_robot_state.lock();
    error.clear();
    robot_state = RobotMoveState::CONSTANT_LOCATION;
    constant_Location_loc = std::pair<float,float>(loc);
    mtx_robot_state.unlock();
}

void RobotFSM::pause()
{
    mtx_robot_state.lock();
    robot_state = RobotMoveState::PAUSE;
    mtx_robot_state.unlock();
}

void RobotFSM::intercept(std::pair<float,float> * loc)
{
    mtx_robot_state.lock();
    error.clear();
    robot_state = RobotMoveState::VARIABLE_LOCATION_INTERCEPT;
    variable_Location_Intercept_loc = loc;
    mtx_robot_state.unlock();
}

void RobotFSM::track(std::pair<float,float> * loc)
{
    mtx_robot_state.lock();
    error.clear();
    robot_state = RobotMoveState::VARIABLE_LOCATION_TRACK;
    variable_Location_Track_loc = loc;
    mtx_robot_state.unlock();
}

void RobotFSM::dribble()
{
    spinner = true;
}

void RobotFSM::stop_dribble()
{
    spinner = false;
}

void RobotFSM::pass()
{

}

void RobotFSM::kick(float kick_speed_x1 = 5, float kick_speed_z1 = 0, int kick_tries1 = 5)
{
    kick_speed_x = kick_speed_x1;
    kick_speed_z = kick_speed_z1;
    kick_tries = kick_tries1;
}

void RobotFSM::send_Command()
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
    
    float veltangent, velnormal, velangular;
    mtx_robot_state.lock();
    switch(robot_state)
    {
        case RobotMoveState::CONSTANT_DIRECTION:
        {
            float angle_error = get_angle() - constant_Direction_dir + 4 * PI;
            while(angle_error > 1 * PI){
                angle_error -= 2*PI;
            }
            veltangent = cos(angle_error*-1)*V_MAX;
            velnormal = sin(angle_error*-1)*V_MAX;
            /*mtx_angle.lock();
            errorIntegral += angle_error;
            float angle_derivative = (angle[0]-angle[1])*60.0;
            const float K_p = -2.0;
            const float K_i = 0.0;
            const float K_d = 0.0;
            velangular = K_p*angle_error+K_i*errorIntegral+K_d*angle_derivative;
            if(id == 0){
                printf("angle: %f  ,angErr: %f , ErrInt: %f, ErrDer: %f, VelAng: %f\n",angle[0], angle_error,errorIntegral,angle_derivative,velangular);
            }
            if(velangular > 2*V_MAX)
            {
                velangular = 2*V_MAX;
            }
            else if (velangular < -2*V_MAX)
            {
                velangular = -2*V_MAX;
            }
            mtx_angle.unlock();*/
            break;
        }
        case RobotMoveState::CONSTANT_LOCATION:
        {
            float xdif = constant_Location_loc.first - get_x();
            float ydif = constant_Location_loc.second - get_y();
            float angle1 = atan2(ydif,xdif);
            float angle_error = get_angle() - angle1 + 4 * PI;
            while(angle_error > 1 * PI){
                angle_error -= 2*PI;
            }
            
            float new_error = sqrt(pow(xdif,2)+pow(ydif,2));
            float error_derivative = 0;
            float error_integral = 0;
            if(error.size() > 0){
                error_derivative = (new_error-error.back())*60;
                for(int i = 0; i < error.size(); i++){
                    error_integral+=error[i];
                }
            }
            const float K_p = 1.0;
            const float K_i = 0.06;
            const float K_d = 0.02;
            float optimal_velocity = K_p*new_error+K_i*error_integral+K_d*error_derivative;
            if(id == 0)
            {
                //printf("P : %f   I: %f    D: %f    Res: %f    Err: %f\n", K_p*new_error,K_i*error_integral,K_d*error_derivative, optimal_velocity, new_error);
            }
            if(optimal_velocity > V_MAX){
                optimal_velocity = V_MAX;
            }else if(optimal_velocity < -V_MAX){
                optimal_velocity = -V_MAX;
            }
            error.push_front(new_error);
            while(error.size() > SIZE)
            {
                error.pop_back();
            }
            veltangent = cos(-1*angle_error)*optimal_velocity;
            velnormal = sin(-1*angle_error)*optimal_velocity;
            //veltangent = 0;
            //velnormal = 0;
            break;
        }
        case RobotMoveState::PAUSE:
        {
            veltangent = 0;
            velnormal = 0;
            if(id == 0 && isYellow){
                //printf("Y: angle: %f\n", angle[0]);
            }else if(id == 0 && !isYellow){
                //printf("B: angle: %f\n", angle[0]);
            }
            break;
        }
        case RobotMoveState::VARIABLE_LOCATION_INTERCEPT:
        {
            veltangent = 0;
            velnormal = 0;
            break;
        }
        case RobotMoveState::VARIABLE_LOCATION_TRACK:
        {
            veltangent = 0;
            velnormal = 0;
            break;
        }
    
    }
    mtx_robot_state.unlock();
    mtx_robot_turn_state.lock();
    switch(robot_turn_state)
    {
        case RobotTurnState::CONSTANT_ANGLE:
        {
            float new_error = get_angle_diff(get_angle(),constant_Angle_angle);
            float error_derivative = 0;
            float error_integral = 0;
            if(!error1.empty())
            {
                error_derivative = (new_error-error[0]) * 60;
                for(int i = 0 ; i < error1.size();i++)
                {
                    //printf("%f ",error1[i]);
                    error_integral += error1[i];
                }
                //printf("\n");
            }
            const float K_p = -1.3;
            const float K_i = -0.06;
            const float K_d = -0.02;
            float optimal_velocity = K_p*new_error+K_i*error_integral+K_d*error_derivative;
            if(id == 0 && isYellow)
            {
                //printf("P : %f   I: %f    D: %f    Res: %f    Err: %f\n", K_p*new_error,K_i*error_integral,K_d*error_derivative, optimal_velocity, new_error);
            }
            if(optimal_velocity > V_ANG_MAX)
            {
                optimal_velocity = V_ANG_MAX;
            }
            else if(optimal_velocity < -V_ANG_MAX)
            {
                optimal_velocity = -V_ANG_MAX;
            }
            velangular = optimal_velocity;
            
            error1.push_front(new_error);
            while(error1.size() > SIZE)
            {
                error1.pop_back();
            }
            break;
        }
        case RobotTurnState::TRACK_OBJECT:
        {
            velangular = 0;
            break;
        }
        case RobotTurnState::DIRECTION_OF_MOVEMENT:
        {
            velangular = 0;
            break;
        }
    }
    mtx_robot_turn_state.unlock();
    command->set_veltangent(veltangent);
    command->set_velnormal(velnormal);
    command->set_velangular(velangular);
    
    //packet.PrintDebugString();

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

void RobotFSM::set_id(int id1)
{
    id = id1;
}
void RobotFSM::set_isYellow(bool isYellow1)
{
    isYellow = isYellow1;
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
    angle1 += 2*PI;
    while(angle1 > 1*PI)
    {
        angle1 -= 2*PI;
    }
    assert(angle1 >= -PI);
    assert(angle1 <= PI);
    angle.push_front(angle1);
    while(angle.size() > SIZE)
    {
        angle.pop_back();
    }
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

std::pair<float,float> RobotFSM::get_speed()
{
    if(x.size() > 1 && y.size() > 1)
    {
        return std::make_pair((x[0]-x[1])*60,(y[0]-y[1])*60);
    }
    else{
        printf("ERROR: Accessing position while position info is empty");
        return std::make_pair(0.0,0.0);
    }
        
}


float get_angle_diff(float angle1, float angle2)
{
    float diff = angle1 - angle2 + 4*PI;
    while(diff > PI)
    {
        diff -= 2*PI;
    }
    return diff;
}
