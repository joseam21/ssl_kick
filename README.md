# robocup_soccer_2020

## Installation
Only works on linux
1. Install grsim through this [link](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md)
2. `mkdir build`
3. `cd build`
4. `cmake ..`
5. `make`
6. `cd ../bin`
7. `./run`
open grsim first
set to 6 robots - in the game tab on the left
Go into the move_robots folder. cmake, make and run move_robots

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
