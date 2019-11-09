#include "moveableobject.h"

float readjust_angle(float angle);

MoveableObject::MoveableObject():hasOrientation(true){}

MoveableObject::MoveableObject(bool hasOrientation1):hasOrientation(hasOrientation1){}

void MoveableObject::update_geometry(float x1, float y1, float angle1, float time1, float confidence1){
    // assert base assumptions
    mtx_time.lock();
    assert(x.size() == y.size());
    assert(y.size() == angle.size());
    assert(angle.size() == time.size());
    if(time.size() > 0 && (time1-time[0])*480 < 1){
        if(confidence1 > confidence){
            x[0] = x1;
            y[0] = y1;
            angle[0] = readjust_angle(angle1);
            time[0] = time1;
            confidence = confidence1;
        }
    }else{
        x.push_front(x1);
        y.push_front(y1);
        angle.push_front(readjust_angle(angle1));
        time.push_front(time1);
        while(x.size() > SIZE){
            x.pop_back();
            y.pop_back();
            angle.pop_back();
            time.pop_back();
        }
    }
    mtx_time.unlock();
}

// This accesses the x deque, but it only takes out a singular element, so no mutex lock necessary
float MoveableObject::get_x(){
    if(x.empty()){
        return 0;
    }
    return x[0];
}
float MoveableObject::get_y(){
    if(y.empty()){
        return 0;
    }
    return y[0];
}
float MoveableObject::get_angle(){
    if(angle.empty()){
        return 0;
    }
    return angle[0];
}
float MoveableObject::get_time(){
    if(time.empty()){
        return 0;
    }
    return time[0];
}

std::pair<float,float> MoveableObject::get_vel(int num){
    mtx_x.lock(); // safe to access multiple x elements
    mtx_y.lock(); // safe to access multiple y elements
    mtx_time.lock(); // safe to access multiple time elements
    assert(num >= 1);
    assert(x.size() == y.size());
    assert(x.size() == time.size());
    if(x.size() > num){
        return std::make_pair((x[0]-x[num])/(time[0]-time[num]),(y[0]-y[num])/(time[0]-time[num]));
    }else{
        return std::make_pair(0,0);
    }
    //obviously unlock them
    mtx_x.unlock();
    mtx_y.unlock();
    mtx_time.unlock();
}

float MoveableObject::get_ang_vel(int num){
    mtx_angle.lock();
    mtx_time.lock();
    assert(num >= 1);
    assert(angle.size() == time.size());
    if(angle.size() > num){
        return (angle[0]-angle[num])/(time[0]-time[num]);
    }else{
        return 0;
    }
    mtx_angle.unlock();
    mtx_time.unlock();
}

std::pair<float,float> MoveableObject::get_loc(){
    assert(x.size() == y.size());
    if(!x.empty()){
        return std::make_pair(x[0],y[0]);
    }else{
        return std::make_pair(0,0);
    }
}

std::pair<float,float> MoveableObject::get_est_loc(float time1, float target_speed, float max_accel){}
std::pair<float,float> MoveableObject::get_est_vel(float time1, float target_speed, float max_accel){}
float MoveableObject::get_est_ang_vel(float time1, float target_ang_vel, float max_accel){}
float MoveableObject::get_est_ang(float time1, float target_ang_vel, float max_accel){}

float readjust_angle(float angle){
    int numoff = (int)(angle/PI);
    if(numoff > 0){
        angle -= ((int)((numoff+1)/2))*2*PI;
    }else if(numoff < 0){
        angle += ((int)((1-numoff)/2))*2*PI;
    }else{
        return angle;
    }
}



