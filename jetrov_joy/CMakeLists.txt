cmake_minimum_required(VERSION 2.8.3)
project(jetrov_joy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
  #DEPENDS system_lib
)

include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
)


#include_directories(include ${catkin_INCLUDE_DIRS})
#add_executable(joystick_controller src/joystick_controller.cpp)
#target_link_libraries(joystick_controller ${catkin_LIBRARIES})
