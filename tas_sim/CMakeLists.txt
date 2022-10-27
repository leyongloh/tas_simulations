cmake_minimum_required(VERSION 2.8.3)
project(tas_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
