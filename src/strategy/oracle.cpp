#include "oracle.h"
#include "basic_offense.h"
#include <math.h>

Oracle::Oracle(){
}

int Oracle::find_posession(){
    pair<float, float> ball_loc = RobotControls::getCurrentBallLoc();
    bool team = true;
    float min_dist = 100000.0;
    int closest_robot = 0;
    for (int n=0; n < 12; n++) {
        if(n >= 6) team = false;
        RobotFSM& robo = RobotControls::getRobot(team, n % 6);
        float x = robo.get_x();
        float y = robo.get_y();
        float dist = pow(ball_loc.first - x, 2) + pow(ball_loc.second - y, 2);
        if (dist < min_dist) {
            min_dist = dist;
            closest_robot = n;
        }
    }
    if(dist < .3){
        return closest_robot;
    }
    else{
        return -1;
    }
}

Play Oracle::choose_play(){
    int hasball = find_posession();
    if(hasball < 6){
    	return BasicOffense(hasball);
    }
    /*else if(hasball == -1){
	    return GetPossession(hasball);
    }
    else{
	    return BasicDefense(hasball);
    }*/
}
