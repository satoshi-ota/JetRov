cmake_minimum_required(VERSION 2.8.3)
project(jetrov_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  jetrov_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/JetrovController.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jetrov_control
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf jetrov_msgs
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(jetrov_control
    src/library/JHPWMPCA9685.cpp
    src/library/steer_controller.cpp
    src/library/speed_controller.cpp
    src/library/dead_reckoning.cpp)

find_library(I2C_LIBRARY NAMES libi2c.so)
target_link_libraries(jetrov_control
    ${catkin_LIBRARIES}
    ${I2C_LIBRARY})

add_dependencies(jetrov_control ${catkin_EXPORTED_TARGETS})

add_executable(jetrov_controller_node src/nodes/jetrov_controller_node.cpp)
add_dependencies(jetrov_controller_node jetrov_control_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(jetrov_controller_node jetrov_control ${catkin_LIBRARIES})

add_executable(dead_reckoning_node src/nodes/dead_reckoning_node.cpp)
add_dependencies(dead_reckoning_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dead_reckoning_node jetrov_control ${catkin_LIBRARIES})
