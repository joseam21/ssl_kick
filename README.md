# robocup_soccer_2020

## Installation
Only works on linux
1. Install grsim through this [link](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md)
2. `cd build`
3. `cmake ..`
4. `make`
5. `./controls_test`
6. open grsim

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
