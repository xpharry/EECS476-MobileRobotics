# lin_steering
Steering algorithm using odometry.  For mobot, "odom" is perfect.  Neede to relax this
assumption.

If start with good initial conditions, linear steering algorithm will do a good job.
Can compare feedback controller to open-loop controller.

## Example usage
Start up gazebo, load the mobot model, desired-state publisher, desired-state client,
and linear-steering algorithm.
`roslaunch gazebo_ros empty_world.launch`
`roslaunch mobot_urdf mobot.launch`
`rosrun mobot_pub_des_state mobot_pub_des_state`
`rosrun my_steering_algorithm_console my_steering_algorithm_console`
`rosrun lin_steering lin_steering_wrt_odom`

or

launch file:
'roslaunch my_lin_steering_console my_lin_steering_console.launch'

    
