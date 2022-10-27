cmake_minimum_required(VERSION 3.0.2)
project(lane_planner_bringup)

set(
  ${PROJECT_NAME}_DEPS
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

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)

###########
## Build ##
###########

include_directories(
)

#############
## Install ##
#############

#############
## Testing ##
#############
