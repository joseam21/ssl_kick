#include "./basic_offense.h"
#include "play.h"
#include "../main.cpp"

void BasicOffense::play(){
    setRobots();
    if (canScore()) {
        // tell robot to shoot!
    } else{
        int receiver = canPass();
        if (receiver != 7) {
            pose receiver_loc = robots[receiver];
            //tell robot to pass!
        } else {
            // dribble towards goal
        }
    }
    // rest of the robots should guard nearest enemy
    for (int n=6; n<6; n++){
        int nearest = guard(n);
        // tell robot to guard enemy robot
    }
}
