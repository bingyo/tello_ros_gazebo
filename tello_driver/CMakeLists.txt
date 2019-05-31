cmake_minimum_required(VERSION 2.8.3)
project(tello_driver)

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS
  rospy
    std_msgs
    geometry_msgs
    sensor_msgs
)

############
## catkin ##
############

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS 
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
)

#############
## Install ##
#############

install(
  TARGETS 
    
  # Libraries

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
