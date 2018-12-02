cmake_minimum_required(VERSION 2.8.3)
project(pca9685_board)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_library(wiringPi_LIB wiringPi)

catkin_package(CATKIN_DEPENDS roscpp message_runtime)

add_message_files(DIRECTORY msg FILES Servo.msg)
generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIR})

add_executable(pca9685_board src/pca9685_board.cpp)
target_link_libraries(pca9685_board ${catkin_LIBRARIES} ${wiringPi_LIB})
