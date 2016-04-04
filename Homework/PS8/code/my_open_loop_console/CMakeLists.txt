cmake_minimum_required(VERSION 2.8.3)
project(my_open_loop_console)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(my_open_loop_console src/my_open_loop_console.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(my_open_loop_console ${catkin_LIBRARIES})


