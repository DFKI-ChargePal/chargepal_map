cmake_minimum_required(VERSION 3.0.2)
project(chargepal_map)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  rospy
  std_msgs
)

catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  DIRECTORY srv
  FILES
  User.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


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
  scripts/start_maps.py
  scripts/manipulation_server_ui.py
  scripts/map_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
