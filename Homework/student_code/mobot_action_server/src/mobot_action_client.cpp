// mobot_action_client: 
// wsn, October, 2014

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mobot_action_server/demoAction.h>
#include <std_msgs/Bool.h>

#define PI 3.1415926

bool g_lidar_alarm=false; // global var for lidar alarm

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
    ROS_INFO("LIDAR alarm received!"); 
  }
  else {
    ROS_INFO("no LIDAR alarm!"); 
  }
} 

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const mobot_action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobot_action_client_node"); // name this node
    ros::NodeHandle n;
    int g_count = 0;
    // here is a "goal" object compatible with the server, as defined in mobot_action_server/action
    mobot_action_server::demoGoal goal; 
    
    // use the name of our server, which is: mobot_action (named in mobot_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<mobot_action_server::demoAction> action_client("mobot_action", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(10.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    while(true) {
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback); 

        goal.distance.resize(5);
        goal.distance[0] = 4; 
        goal.distance[1] = 2; 
        goal.distance[2] = 4; 
        goal.distance[3] = 2; 
        goal.distance[4] = 4;

        goal.angle.resize(5);
        goal.angle[0] = 4;
        goal.angle[1] = PI/2;
        goal.angle[2] = PI/2;
        goal.angle[3] = -PI/2;
        goal.angle[4] = -PI/2;

        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    }

    return 0;
}

