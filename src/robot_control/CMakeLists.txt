cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
#  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/robot_control_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

