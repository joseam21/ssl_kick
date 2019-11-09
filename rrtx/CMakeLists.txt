cmake_minimum_required(VERSION 3.15)
project(RRTXSimpleVisualization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)

add_executable(RRTXSimpleVisualization main.cpp Visualizer.cpp Visualizer.h RRTX.cpp RRTX.h Node.cpp Node.h)

target_link_libraries(RRTXSimpleVisualization Qt5::Widgets)