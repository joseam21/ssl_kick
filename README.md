# robocup_soccer_2020

## Installation
Only works on linux
Install grsim
cmake, make and run grsim from the bin folder
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
