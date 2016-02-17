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
    //ROS_INFO("no LIDAR alarm!"); 
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

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  //g_goal_active=true; //let main() know that the server responded that this goal is in process
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const mobot_action_server::demoFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback");
    ROS_INFO("feedback status No. %d",fdbk_msg->fdbk);
    //g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
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
    bool server_exists = action_client.waitForServer(ros::Duration(1000.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback);

    ros::Rate loop_timer(100);
    while(!g_lidar_alarm) {
        goal.input = 1;

        goal.distance.resize(5);
        goal.distance[0] = 4; 
        goal.distance[1] = 2; 
        goal.distance[2] = 4; 
        goal.distance[3] = 2; 
        goal.distance[4] = 5;

        goal.angle.resize(5);
        goal.angle[0] = 0;
        goal.angle[1] = PI/2;
        goal.angle[2] = PI/2;
        goal.angle[3] = -PI/2;
        goal.angle[4] = -PI/2;

        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        //action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        ros::spinOnce();
        loop_timer.sleep();
    }

    return 0;
}

