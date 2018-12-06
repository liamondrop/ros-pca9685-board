cmake_minimum_required(VERSION 2.8.3)
project(pca9685_board)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wpedantic")

find_library(wiringPi_LIB wiringPi)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_message_files(FILES Servo.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

set(SOURCES
    src/pca9685_board_node.cpp
    src/pca9685_board.cpp
)

add_executable(pca9685_board ${SOURCES})
target_link_libraries(pca9685_board
    ${catkin_LIBRARIES}
    ${wiringPi_LIB}
)
