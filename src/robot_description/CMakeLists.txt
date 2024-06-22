cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
)

catkin_package(
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Install other files (e.g., launch and config files)
install(DIRECTORY config  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install YAML files in the 'config' directory
install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)


