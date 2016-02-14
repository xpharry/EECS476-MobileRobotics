cmake_minimum_required(VERSION 2.8.3)
project(my_ros_service)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(my_ros_service src/my_ros_service.cpp)
cs_add_executable(my_ros_client src/my_ros_client.cpp)
cs_add_executable(my_path_service src/my_path_service.cpp)
cs_add_executable(my_path_service2 src/my_path_service2.cpp)
#cs_add_executable(path_service_soln src/path_service_soln.cpp)
cs_add_executable(my_path_client src/my_path_client.cpp)
cs_install()
cs_export()
    
