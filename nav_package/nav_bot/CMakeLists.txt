cmake_minimum_required(VERSION 3.0.2)
project(nav_bot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  amcl
  fake_localization
  geometry_msgs
  map_server
  message_generation
  move_base
  sensor_msgs
  std_msgs
  roscpp
  rospy
)


################################################################################
# Setup for python modules and scripts
################################################################################


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Task.msg
  TaskStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES 
  CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs std_msgs message_runtime rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

# Mark config/include/launch directory for installation
install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitignore" EXCLUDE
)
