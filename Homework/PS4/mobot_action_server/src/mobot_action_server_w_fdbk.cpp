#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mobot_action_server/demoAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

int g_count = 0;
bool g_count_failure = false;
bool g_lidar_alarm=false; // global var for lidar alarm

//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.5; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;

//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


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


class MobotActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in mobot_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<mobot_action_server::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    mobot_action_server::demoGoal goal_; // goal message, received from client
    mobot_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    mobot_action_server::demoFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    MobotActionServer(); //define the body of the constructor outside of class definition

    ~MobotActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<mobot_action_server::demoAction>::GoalConstPtr& goal);

    // +++++++++++++++++
    // here are a few useful utility functions:
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    void do_inits(ros::NodeHandle &n);
    void do_halt();
    void do_spin(double spin_ang);
    void do_move(double distance);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class MobotActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

MobotActionServer::MobotActionServer() :
   as_(nh_, "mobot_action", boost::bind(&MobotActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "mobot_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of MobotActionServer...");
    // do any other desired initializations here...specific to your implementation
	do_inits(nh_);
    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <mobot_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "mobot_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void MobotActionServer::executeCB(const actionlib::SimpleActionServer<mobot_action_server::demoAction>::GoalConstPtr& goal) {
    ROS_INFO("in MobotActionServer::executeCB");
    std::vector<double> spin_angle = goal->angle;
    std::vector<double> travel_distance = goal->distance;

    int num_angle = spin_angle.size();
    // for(int i = 0; i < num_angle; i++) { 
    // 	ROS_INFO("angle[%d] = %f", i, spin_angle[i]);
    // }
    // int num_distance = travel_distance.size();
    // for(int i = 0; i < num_distance; i++) { 
    //     ROS_INFO("distance[%d] = %f", i, travel_distance[i]);
    // }

    //do work here: this is where your interesting code goes
    ros::Rate timer(10.0); // 1Hz timer
    countdown_val_ = goal->input;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (countdown_val_>0) {
       ROS_INFO("countdown = %d",countdown_val_);

	    feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
   		as_.publishFeedback(feedback_);
	     // excute the movement
	    for(int i = 0; i < num_angle; i++) {
	        do_spin(spin_angle[i]); // carry out this incremental action
	        do_move(travel_distance[i]); // carry out this incremental action
	        ROS_INFO("spin_angle = %f", spin_angle[i]);
	        ROS_INFO("travel_distance = %f", travel_distance[i]);
	    }
 
	    do_halt();

		ros::Subscriber alarm_subscriber = nh_.subscribe("lidar_alarm",1,alarmCallback);
       // each iteration, check if cancellation has been ordered
       if (g_lidar_alarm){	
          ROS_WARN("goal cancelled!");
          do_halt();
          result_.output = countdown_val_;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 		}
 	
 	   //if here, then goal is still valid; provide some feedback
 	   // feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	   // as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
       countdown_val_--; //decrement the timer countdown
       timer.sleep(); //wait 1 sec between loop iterations of this timer
    }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = countdown_val_; //value should be zero, if completed countdown

    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}


//signum function: strip off and return the sign of the argument
double MobotActionServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double MobotActionServer::min_spin(double spin_angle) {
    if (spin_angle > M_PI) {
        spin_angle -= 2.0*M_PI;}
    if (spin_angle < -M_PI) {
        spin_angle += 2.0*M_PI;}
    return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double MobotActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion MobotActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void MobotActionServer::do_inits(ros::NodeHandle &n) {
    //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
}

void MobotActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<100;i++) {
        g_twist_commander.publish(g_twist_cmd);
        loop_timer.sleep(); 
    }   
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void MobotActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.linear.x = 0.2;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
    }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void MobotActionServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
        ros::Subscriber alarm_subscriber = nh_.subscribe("lidar_alarm",1,alarmCallback);
        if(g_lidar_alarm) {
            ROS_WARN("lidar_alarm activated! too close... mission canceled!");
            //do_halt();
            break;
        }
        g_twist_commander.publish(g_twist_cmd);
        timer+=g_sample_dt;
        loop_timer.sleep(); 
    }  
    do_halt();
}

// ============ main function ===========

int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    MobotActionServer as_object; // create an instance of the class "MobotActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under mobot_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

