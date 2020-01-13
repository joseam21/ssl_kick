cmake_minimum_required (VERSION 2.8)

project (move_bots)

# Protobuf
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/proto/grSim_Commands.proto
    src/proto/grSim_Packet.proto
    src/proto/grSim_Replacement.proto
    src/proto/messages_robocup_ssl_detection.proto
    src/proto/messages_robocup_ssl_geometry.proto
    src/proto/messages_robocup_ssl_geometry_legacy.proto
    src/proto/messages_robocup_ssl_refbox_log.proto
    src/proto/messages_robocup_ssl_wrapper.proto
    src/proto/messages_robocup_ssl_wrapper_legacy.proto
)
message ("PROTO_CPP = ${PROTO_CPP}")
message ("PROTO_H = ${PROTO_H}")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OMPL library
find_package(OMPL REQUIRED)
if (NOT OMPL_FOUND)
    message(SEND_ERROR "Failed to find OMPL! Install OMPL and try again!")
endif ()

include_directories(${PROJECT_SOURCE_DIR}/src/controls)

set (CNTRLS_SRCS ${CNTRLS_SRCS}
        src/controls/logger.cpp
        src/controls/moveableobject.cpp
        src/controls/network.cpp
        src/controls/robotFSM.cpp
        src/controls/netraw.cpp
        src/controls/robocup_ssl_client.cpp
        src/controls/robotcontrols.cpp
        src/controls/logger.cpp
)

set (STRATEGY_SRCS ${STRATEGY_SRCS}
  src/strategy/basic_offense.cpp
  src/strategy/oracle.cpp
  src/strategy/play.cpp
)

set (PLANNING_SRCS ${PLANNING_SRCS}
  src/path_planner/RRTX.cpp
  src/path_planner/Node.cpp
)

# executable for controls testing
add_executable(controls_test src/controls/main.cpp ${CNTRLS_SRCS} ${PROTO_CPP} ${PROTO_H})

target_include_directories(controls_test
    PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# link the exe against the libraries
target_link_libraries(controls_test
    PUBLIC
    ${PROTOBUF_LIBRARIES}
)

# executable for strategy testing
add_executable(strategy_test src/strategy/strategy_main.cpp ${STRATEGY_SRCS} ${CNTRLS_SRCS} ${PROTO_CPP} ${PROTO_H})

target_include_directories(strategy_test
    PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# link the exe against the libraries
target_link_libraries(strategy_test
    PUBLIC
    ${PROTOBUF_LIBRARIES}
)

# executable for path planning testing
add_executable(planning_test src/path_planner/grsim_plan.cpp ${CNTRLS_SRCS} ${PLANNING_SRCS} ${PROTO_CPP} ${PROTO_H})

target_include_directories(planning_test
    PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# link the exe against the libraries
target_link_libraries(planning_test
    PUBLIC
    ${PROTOBUF_LIBRARIES}
    ${OMPL_LIBRARIES}
)
