# robocup_soccer_2020

## Installation
Only works on linux
1. Install grsim through this [link](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md)
2. Go back to robocup_soccer_2020:
3. `cd build`
4. `cmake ..`
5. `make`
6. `./controls_test`
7. open grsim

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
