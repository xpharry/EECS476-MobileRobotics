# example use

****************************************
# gmapping, wandering

roslaunch gazebo_ros empty_world.launch

roslaunch mobot_urdf mobot_startup_open_loop.launch

rosbag record -O mapData /scan /tf

rosrun my_open_loop_console my_open_loop_console

****************************************
# gmapping, saving data

roscore

rosrun gmapping slam_gmapping scan :=/ scan

rosbag play my_mapData.bag

rosrun rviz rviz

rosrun map_server map_saver -f my_gmapping

****************************************
# amcl

roslaunch gazebo_ros empty_world.launch

roslaunch mobot_urdf mobot_startup_open_loop.launch

roscd my_map

	rosrun map_server map_server my_gmapping.yaml

rosrun amcl amcl

rosrun my_open_loop_console my_open_loop_console

=== or ===

use launch file:

	roslaunch my_open_loop_console my_robot_moving_in_known_map.launch
