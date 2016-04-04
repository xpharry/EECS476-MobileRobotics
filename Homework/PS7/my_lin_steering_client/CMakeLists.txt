cmake_minimum_required(VERSION 2.8.3)
project(my_lin_steering_client)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(my_lin_steering_client src/my_lin_steering_client.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(my_lin_steering_client ${catkin_LIBRARIES})


