cmake_minimum_required(VERSION 2.8.3)
project(jetrov_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

 add_message_files(
   FILES
   PulseCount.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
