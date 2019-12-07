cmake_minimum_required(VERSION 3.15)
project(RRTXOptimizedVisualization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -L /usr/local/lib")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)


# OMPL library
find_package(OMPL REQUIRED)
if (NOT OMPL_FOUND)
    message(SEND_ERROR "Failed to find OMPL! Install OMPL and try again!")
endif ()
# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
)

add_executable(RRTXOptimizedVisualization main.cpp Node.cpp Node.h RRTX.cpp RRTX.h Visualizer.cpp Visualizer.h)

target_link_libraries(RRTXOptimizedVisualization Qt5::Widgets boost_system ${OMPL_LIBRARIES})