cmake_minimum_required(VERSION 2.8.3)
project(pca9685_board)

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

add_executable(pca9685_board src/pca9685_board.cpp)
target_link_libraries(pca9685_board
    ${catkin_LIBRARIES}
    ${wiringPi_LIB}
)
