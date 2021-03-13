# kick_dev

## Installation
Only works on linux
1. Install grsim through this [link](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md)
2. Go back to robocup_soccer_2020:
3. `cd build`
4. `cmake ..`
5. `make`
6. `./controls_test`
7. open grsim


(NOTE: if `make` fails, you likely need to install OMPL. Follow the README here: https://github.mit.edu/MIT-Robotics-Team/robocup_soccer_2020/tree/master/src/path_planner)

## Structure
* path_planner
  * GetPath(state, goal, path)
* strategy
  * RunPlay
* controls
  * MoveToLocation(location)
  * GetState(state)
  * TrackTrajectory(path)
* vision
  * TBD
* kick_dev
