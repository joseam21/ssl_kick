cmake_minimum_required (VERSION 3.1)
project (RoboCup_Strategy)

# The version number.
set (CMAKE_CXX_STANDARD 11)
set (RoboCup_Strategy_VERSION_MAJOR 1)
set (RoboCup_Strategy_VERSION_MINOR 0)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}")
 
# add the executable
add_executable(strategy basic_offense.cpp)

