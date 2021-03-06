#include "./basic_offense.h"
#include "play.h"
#include "../../move_robots/robotFSM.cpp"
#include "../main.cpp"

void BasicOffense::play(){
    setRobots();
    if (canScore()) {
        // tell robot to shoot!
    } else{
        int receiver = canPass();
        if (receiver != 7) {
            RobotFSM receiver_loc = controller.getRobot(true, receiver);
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
