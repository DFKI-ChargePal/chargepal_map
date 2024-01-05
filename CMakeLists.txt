cmake_minimum_required(VERSION 3.0.2)
project(chargepal_map)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chargepal_map
#  CATKIN_DEPENDS actionlib actionlib_msgs rospy std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/manipulation_server.py
  scripts/state_machine_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
