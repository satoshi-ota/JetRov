cmake_minimum_required(VERSION 2.8.3)
project(jetrov_joy_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  jetrov_msgs
  jetrov_control
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs jetrov_msgs jetrov_control
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(joystick_control_node src/joystick_control_node.cpp)
target_link_libraries(joystick_control_node ${catkin_LIBRARIES})
