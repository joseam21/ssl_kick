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
add_executable(move_bots move.cpp network.cpp robotFSM.cpp netraw.cpp robocup_ssl_client.cpp main.cpp ${PROTO_CPP} ${PROTO_H})

target_include_directories(move_bots
    PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# link the exe against the libraries
target_link_libraries(move_bots
    PUBLIC
    ${PROTOBUF_LIBRARIES}
)
