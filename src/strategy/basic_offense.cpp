#include "./basic_offense.h"

using namespace std;

// BasicOffense (RobotControls controls, int hasBall) : Play(controls, hasBall){
//             controller = controls;
//             posession = hasBall;
//         };

void BasicOffense::play(){
    RobotFSM attacker = RobotControls::getRobot(true, posession);

    if (canScore()) {
        attacker.kick(goal_loc.first, goal_loc.second, 1);
    } else{
        int receiver = canPass();
        if (receiver != 7) {
            RobotFSM receiver_loc = RobotControls::getRobot(true, receiver);
            attacker.kick(receiver_loc.get_x(), receiver_loc.get_y(), 1);
        } else {
            attacker.dribble();
        }
    }
    // rest of the robots should guard nearest enemy
    for (int n=6; n<6; n++){
        if (n != posession){
            RobotFSM guarder = RobotControls::getRobot(true, n);
            int nearest = guard(n);
            RobotFSM other = RobotControls::getRobot(false, nearest);
            std::pair<float, float> location = std::make_pair(other.get_x(), other.get_y());
            guarder.move_to_track(&location);
        }
    }
}
