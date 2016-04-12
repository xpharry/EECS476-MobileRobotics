#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mobot_action_server/pathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

const double PI = 3.1415926;
bool is_alldone = 1;

actionlib::SimpleActionClient<mobot_action_server::pathAction> *action_client;
ros::Subscriber alarm_subscriber;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const mobot_action_server::pathResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO(" got result is_halt = %d", result->is_alldone.data);
    is_alldone = result->is_alldone.data;
}

void activeCb() {
    ROS_INFO("Goal just went active");
}

void feedbackCb(const mobot_action_server::pathFeedbackConstPtr& feedback) {
    ROS_INFO("Got Feedback ... ");
    ROS_INFO("des_pose: %f, %f; is_finished: %d", feedback->des_pose.position.x, feedback->des_pose.position.y, feedback->is_finished.data);
}

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void alarmCallback(const std_msgs::Bool& alarm_msg) { 
    bool g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    mobot_action_server::pathGoal goal;
    ROS_INFO("LIDAR alarm = %d", g_lidar_alarm);
    if (g_lidar_alarm) {
        ROS_INFO("LIDAR alarm received!");
        action_client->cancelGoal();

        ROS_INFO("turning around!");
        //commandSquarePath();
        std::vector<geometry_msgs::PoseStamped> poses(1);
        poses.at(0).pose.position.x = 0;
        poses.at(0).pose.position.y = 0;
        poses.at(0).pose.orientation = convertPlanarPhi2Quaternion(PI/2);

        goal.path.poses = poses;
        action_client->sendGoal(goal);
        // action_client->waitForResult(); // wait forever... 

        poses.resize(4);

        poses.at(0).pose.position.x = 0;
        poses.at(0).pose.position.y = 3;

        poses.at(1).pose.position.x = 0;
        poses.at(1).pose.position.y = 3;

        poses.at(2).pose.position.x = 0;
        poses.at(2).pose.position.y = 3;

        poses.at(3).pose.position.x = 0;
        poses.at(3).pose.position.y = 3;

        goal.path.poses = poses;
        // action_client->sendGoal(goal);
        action_client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    }
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "path_action_client_node"); // name this node
        ros::NodeHandle nh;

        alarm_subscriber = nh.subscribe("/lidar_alarm", 1, alarmCallback);

        // here is a "goal" object compatible with the server, as defined in mobot_action_server/action
        mobot_action_server::pathGoal goal; 
        
        // use the name of our server, which is: example_action (named in mobot_action_server.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        action_client = new actionlib::SimpleActionClient<mobot_action_server::pathAction>("mobot_action", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client->waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        // stuff a goal message:
        geometry_msgs::Quaternion quat;
        quat = convertPlanarPhi2Quaternion(0);
        std::vector<geometry_msgs::PoseStamped> poses(4);

        poses.at(0).pose.position.x = 0;
        poses.at(0).pose.position.y = 3;
        poses.at(0).pose.orientation = quat;

        poses.at(1).pose.position.x = 0;
        poses.at(1).pose.position.y = 3;

        poses.at(2).pose.position.x = 0;
        poses.at(2).pose.position.y = 3;

        poses.at(3).pose.position.x = 0;
        poses.at(3).pose.position.y = 3;

        goal.path.poses = poses;
        // action_client->sendGoal(goal);
        action_client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        // action_client->waitForResult(); // wait forever... 

        ros::spin();

    return 0;
}

