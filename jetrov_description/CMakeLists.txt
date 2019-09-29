cmake_minimum_required(VERSION 2.8.3)
project(marin_description)
find_package(catkin REQUIRED COMPONENTS
  urdf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
