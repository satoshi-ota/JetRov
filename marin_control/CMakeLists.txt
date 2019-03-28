cmake_minimum_required(VERSION 2.8.3)
project(marin_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  marin_msgs
)

 generate_messages(
   DEPENDENCIES marin_msgs
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )


catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES marin_control
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf marin_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(marin_imu_node src/marin_imu_node.cpp)
target_link_libraries(marin_imu_node ${catkin_LIBRARIES})

add_executable(marin_odometry_publisher src/marin_odometry_publisher.cpp)
target_link_libraries(marin_odometry_publisher ${catkin_LIBRARIES})

add_executable(marin_state_publisher src/marin_state_publisher.cpp)
target_link_libraries(marin_state_publisher ${catkin_LIBRARIES})

add_executable(marin_velocity_controller src/marin_velocity_controller.cpp)
target_link_libraries(marin_velocity_controller ${catkin_LIBRARIES})
