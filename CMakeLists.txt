cmake_minimum_required(VERSION 2.8.3)
project(pca9685_board)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_library(wiringPi_LIB wiringPi)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_joy
#  CATKIN_DEPENDS joy roscpp turtlesim
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pca9685_board src/pca9685_board.cpp)
target_link_libraries(pca9685_board ${catkin_LIBRARIES} ${wiringPi_LIB})
add_executable(servo src/servo.c src/pca9685.c)
target_link_libraries(servo ${wiringPi_LIB})
