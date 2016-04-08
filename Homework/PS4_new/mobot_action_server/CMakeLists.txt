cmake_minimum_required(VERSION 2.8.3)
project(mobot_action_server)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(mobot_action_server src/mobot_action_server.cpp)
cs_add_executable(mobot_action_client src/mobot_action_client.cpp)

cs_install()
cs_export()
    
