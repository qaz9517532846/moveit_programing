cmake_minimum_required(VERSION 3.0.2)
project(moveit_programing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_move_group
  moveit_ros_planning_interface
  roscpp
  tf
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(programing_main src/programing_main.cpp src/programing_algorithm.cpp)
target_link_libraries(programing_main ${catkin_LIBRARIES})
