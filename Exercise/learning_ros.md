# STDR

	described in:

		 http://wiki.ros.org/stdr_simulator

	command a robot to move, format:

		$ rostopic pub -r 2 /robot0/cmd_vel geometry_msgs/Twist '[0.5, 0, 0]' '[0, 0, 0]'
