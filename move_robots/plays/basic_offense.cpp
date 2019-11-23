#include "./basic_offense.h"

using namespace std;

void BasicOffense::play(){
    RobotFSM attacker = controller.getRobot(true, posession);

    if (canScore()) {
        attacker.kick(get<0>(goal_loc), get<1>(goal_loc), 1);
    } else{
        int receiver = canPass();
        if (receiver != 7) {
            RobotFSM receiver_loc = controller.getRobot(true, receiver);
            attacker.kick(receiver_loc.get_x(), receiver_loc.get_y(), 1);
        } else {
            attacker.dribble();
        }
    }
    // rest of the robots should guard nearest enemy
    for (int n=6; n<6; n++){
        if (n != posession){
            RobotFSM guarder = controller.getRobot(true, n);
            int nearest = guard(n);
            RobotFSM other = controller.getRobot(false, nearest);
            std::pair<float, float> location = std::make_pair(other.get_x(), other.get_y());
            guarder.move_to_track(&location);
        }
    }
}
