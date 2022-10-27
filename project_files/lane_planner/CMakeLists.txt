cmake_minimum_required(VERSION 3.0.2)
project(lane_planner)

# Activate Debugging
set(PRINT_DEBUG ON)

add_compile_options(-std=c++17)

set(
  ${PROJECT_NAME}_DEPS
  angles
  base_local_planner
  control_msgs
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  jsk_rviz_plugins
  message_generation
  message_runtime
  move_base_msgs
  nav_core
  pluginlib
  ros_numpy
  roscpp
  rospack
  rospy
  std_msgs
  tf
)

find_package(
  catkin REQUIRED 
  COMPONENTS ${${PROJECT_NAME}_DEPS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Car_scan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/LaneGlobalPlanner.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)

###########
## Build ##
###########

if(PRINT_DEBUG)
  message(STATUS "Activate PRINT_DEBUG")
  add_definitions(-DPRINT_DEBUG)
endif()

include_directories(
  include ${catkin_INCLUDE_DIRS} 
)

# Lib files
add_library(
  ${PROJECT_NAME}
  src/lib/decision_helper.cpp
  src/lib/lane_global_planner.cpp
  src/lib/trajectory_helper.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Plugin files
add_library(
  ${PROJECT_NAME}_lib
  src/lane_global_planner_ros.cpp
)

add_dependencies(
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  PROGRAMS
  scripts/LaserChecker.py
  scripts/WiiControllerSim.py
  scripts/ZoneVisualisation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY
  cfg 
  launch 
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
  ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  # Test modules
  set(TEST_MODULES 
    trajectory_helper
    trajectory_environment
  )

  foreach(M ${TEST_MODULES}) 
    add_rostest_gtest(${PROJECT_NAME}_${M}_test 
                      test/${M}/launch.test 
                      test/${M}/tests.cpp) 
    
    target_link_libraries(${PROJECT_NAME}_${M}_test ${PROJECT_NAME})
  endforeach()
endif()
