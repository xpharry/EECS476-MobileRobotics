#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <std_msgs/Bool.h> // boolean message 

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher lidar_alarm_publisher = nh.advertise<std_msgs::Bool>("/lidar_alarm", 1);

    while(ros::ok()) {
        std_msgs::Bool lidar_alarm_msg;
        lidar_alarm_msg.data = 1;
        lidar_alarm_publisher.publish(lidar_alarm_msg);
        ros::spinOnce();
    }

    return 0; // should never get here, unless roscore dies
}

