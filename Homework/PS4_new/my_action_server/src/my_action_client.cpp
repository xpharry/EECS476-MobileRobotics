#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/demoAction.h>
#include <std_msgs/Bool.h>


bool g_lidar_alarm = false; // global var for lidar alarm

const double PI = 3.1415926;

class ActionClient {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionClient<my_action_server::demoAction> ac_;
    ros::Subscriber alarm_subscriber;

    // here are some message types to communicate with our server
    my_action_server::demoGoal goal_; // goal message, received from client
    my_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::demoFeedback feedback_; // for feedback 


public:
    ActionClient(); //define the body of the constructor outside of class definition
    ~ActionClient(void) {
    }

    void connectToServer();
    void actionDoneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::demoResultConstPtr& result);
    void commandSquarePath();
    void alarmCallback(const std_msgs::Bool& alarm_msg);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
};


ActionClient::ActionClient() : ac_(nh_, "my_lidar_mobot_action", true) {
    ROS_INFO("in constructor of ActionClient...");
    // do any other desired initializations here...specific to your implementation
    alarm_subscriber = nh_.subscribe("lidar_alarm",1,&ActionClient::alarmCallback,this);
    connectToServer();
}


void ActionClient::connectToServer() {
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = ac_.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = ac_.waitForServer(); //wait forever

    while (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        server_exists = ac_.waitForServer(ros::Duration(5.0));
    }
    
    ROS_INFO("connected to action server");  // if here, then we connected to the server;
}


void ActionClient::actionDoneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}


void ActionClient::commandSquarePath() {
    // stuff a goal message:
    nav_msgs::Path path;
    geometry_msgs::Quaternion quat;
    quat = convertPlanarPhi2Quaternion(0);
    std::vector<geometry_msgs::PoseStamped> poses(4);

    poses.at(0).pose.position.x = 0;
    poses.at(0).pose.position.y = 1;
    poses.at(0).pose.orientation = quat;

    poses.at(1).pose.position.x = 0;
    poses.at(1).pose.position.y = 2;

    poses.at(2).pose.position.x = 0;
    poses.at(2).pose.position.y = 3;

    poses.at(3).pose.position.x = 0;
    poses.at(3).pose.position.y = 4;

    path.poses = poses;
    goal_.path = path; // this merely sequentially numbers the goals sent
    ac_.sendGoal(goal_, boost::bind(&ActionClient::actionDoneCb, this, _1, _2)); // simple example--send goal, but do not specify callbacks
    //ac_.sendGoal(goal_,&ActionClient::actionDoneCb); // we could also name additional callback functions here, if desired
    //    ac_.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    ac_.waitForResult();
    // bool finished_before_timeout = ac_.waitForResult(ros::Duration(5.0));
    // //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    // while (!finished_before_timeout) {
    //     ROS_WARN("giving up waiting for commandSquarePath ... ");
    //     finished_before_timeout = ac_.waitForResult(ros::Duration(5.0));
    // }
    // ROS_INFO("commandSquarePath sent... ");
}


void ActionClient::alarmCallback(const std_msgs::Bool& alarm_msg) { 
    g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    ROS_INFO("LIDAR alarm = %d", g_lidar_alarm);
    if (g_lidar_alarm) {
        ROS_INFO("LIDAR alarm received!");
        ac_.cancelGoal();
        std::vector<geometry_msgs::PoseStamped> poses(1);
        // poses.at(0).pose.position.x = 0;
        // poses.at(0).pose.position.y = 0;
        // poses.at(0).pose.orientation = convertPlanarPhi2Quaternion(0);
        // ac_.sendGoal(goal_);      
        // goal_.path.poses = poses; // this merely sequentially numbers the goals sent
        // ac_.waitForResult(); // wait forever...
        
        // bool finished_before_timeout = ac_.waitForResult(ros::Duration(5.0));
        // //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        // while (!finished_before_timeout) {
        //     ROS_WARN("giving up waiting for cancelGoal ... ");
        //     finished_before_timeout = ac_.waitForResult(ros::Duration(5.0));
        // }

        ROS_INFO("turning around!");
        //commandSquarePath();
        //std::vector<geometry_msgs::PoseStamped> poses(1);
        poses.at(0).pose.position.x = 0;
        poses.at(0).pose.position.y = 0;
        poses.at(0).pose.orientation = convertPlanarPhi2Quaternion(1.57);
        ac_.sendGoal(goal_);      
        goal_.path.poses = poses; // this merely sequentially numbers the goals sent
        ac_.waitForResult(); // wait forever... 
    }
}


geometry_msgs::Quaternion ActionClient::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_client"); // name this node
    ROS_INFO("====================== instantiating the my_action_clent: ======================");

    ActionClient ac_object; // create an instance of the class "ActionServer"
    
    ROS_INFO("going into spin");

    while(ros::ok()) {
        ac_object.commandSquarePath();
        ros::spinOnce();
    }

    return 0;
}

