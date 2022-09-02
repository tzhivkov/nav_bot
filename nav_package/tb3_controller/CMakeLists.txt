cmake_minimum_required(VERSION 3.0.2)
project(tb3_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  turtlebot3_msgs
  smach
  smach_ros
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_controller
#  CATKIN_DEPENDS actionlib move_base rospy
#  DEPENDS system_lib
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


catkin_install_python(PROGRAMS
  scripts/fsm_actions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)