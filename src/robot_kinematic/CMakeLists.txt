cmake_minimum_required(VERSION 3.0.2)
project(robot_kinematic)

# Use C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

# Catkin-specific configuration
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(Odometry
  src/Odometry.cpp
)

add_executable(InvKinematic
  src/InvKinematic.cpp
)

# Specify libraries to link with the executable
target_link_libraries(Odometry
  ${catkin_LIBRARIES}
)

target_link_libraries(InvKinematic
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS Odometry InvKinematic
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

