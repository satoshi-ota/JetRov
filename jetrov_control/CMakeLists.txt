cmake_minimum_required(VERSION 2.8.3)
project(jetrov_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jetrov_control
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
    DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(jetrov_control
    src/library/JHPWMPCA9685.cpp
    src/library/speed_controller.cpp)

target_link_libraries(jetrov_control ${catkin_LIBRARIES})
add_dependencies(jetrov_control ${catkin_EXPORTED_TARGETS})

add_executable(jetrov_controller_node src/nodes/jetrov_controller_node.cpp)
add_dependencies(jetrov_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(jetrov_controller_node jetrov_control ${catkin_LIBRARIES})
