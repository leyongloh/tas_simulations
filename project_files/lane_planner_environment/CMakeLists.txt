cmake_minimum_required(VERSION 3.0.2)
project(lane_planner_environment)

set(
  ${PROJECT_NAME}_DEPS
  roscpp
  rospy
  std_msgs
)

find_package(
  catkin REQUIRED 
  COMPONENTS ${${PROJECT_NAME}_DEPS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  PROGRAMS
  scripts/CatmullSpline.py
  scripts/generate_trajectories.py
  scripts/obstacles_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  launch 
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
