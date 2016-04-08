#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mobot_action_server/pathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
const double PI = 3.1415926;

class MobotActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<mobot_action_server::pathAction> as_;
    
    // here are some message types to communicate with our client(s)
    mobot_action_server::pathGoal goal_; // goal message, received from client
    mobot_action_server::pathResult result_; // put results here, to be sent back to the client when done w/ goal
    mobot_action_server::pathFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;

    ros::Publisher twist_commander_;
    geometry_msgs::Twist twist_cmd_;
    geometry_msgs::Pose current_pose_;
    // *******************************
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    void do_spin(double spin_ang);
    void do_move(double distance);
    void do_halt();
    void do_inits(ros::NodeHandle &n);
    void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading);
    // *******************************

public:
    MobotActionServer(ros::NodeHandle nh); //define the body of the constructor outside of class definition

    ~MobotActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<mobot_action_server::pathAction>::GoalConstPtr& goal);
};


MobotActionServer::MobotActionServer(ros::NodeHandle nh) :
    nh_(nh),
    as_(nh_, "mobot_action", boost::bind(&MobotActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of MobotActionServer...");
    // do any other desired initializations here...specific to your implementation

    do_inits(nh_);

    as_.start(); //start the server running
}


void MobotActionServer::executeCB(const actionlib::SimpleActionServer<mobot_action_server::pathAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    nav_msgs::Path path = goal->path;
    ros::Rate timer(1.0); // 1Hz timer
    // ***************************************************
    double spin_angle;
    double travel_distance;
    geometry_msgs::Pose pose_desired;
    int npts = path.poses.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    for (int i = 0; i < npts; i++) { //visit each subgoal
       // each iteration, check if cancellation has been ordered
        if (as_.isPreemptRequested()){ 
          ROS_WARN("goal cancelled!");
          result_.is_halt.data = true;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = path.poses[i].pose; //get first pose from vector of poses
        feedback_.des_pose = pose_desired;
        feedback_.is_finished.data = false; // populate feedback message with current countdown value
        as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal

        get_yaw_and_dist(current_pose_, pose_desired, travel_distance, spin_angle);

        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        current_pose_.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
        
        do_move(travel_distance); // carry out this incremental action
        current_pose_.position = pose_desired.position; // assumes got to desired orientation precisely
        feedback_.is_finished.data = true; // populate feedback message with current countdown value
        as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal

        ros::spinOnce();

        timer.sleep();      
    }
    // ***************************************************
    result_.is_halt.data = false; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

// **************************************************************************************

//signum function: strip off and return the sign of the argument
double MobotActionServer::sgn(double x) {
    if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double MobotActionServer::min_spin(double spin_angle) {
    if (spin_angle>M_PI) {
        spin_angle -= 2.0*M_PI;}
    if (spin_angle< -M_PI) {
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

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void MobotActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    twist_cmd_.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          twist_commander_.publish(twist_cmd_);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
    }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void MobotActionServer::do_move(double distance) {  // always assumes robot is already oriented properly
                                                    // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    twist_cmd_.angular.z = 0.0; //stop spinning
    twist_cmd_.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          twist_commander_.publish(twist_cmd_);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
    }  
    do_halt();
}

void MobotActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    twist_cmd_.angular.z= 0.0;
    twist_cmd_.linear.x=0.0;
    for (int i=0;i<10;i++) {
        twist_commander_.publish(twist_cmd_);
        loop_timer.sleep(); 
    }   
}

void MobotActionServer::do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;    
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;  
    
    //define initial position to be 0
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_pose_.position.z = 0.0;
    
    // define initial heading to be "0"
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;
    current_pose_.orientation.w = 1.0;
    
    twist_commander_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);    
}


void MobotActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading) {
    double x_diff = goal_pose.position.x - current_pose.position.x;
    double y_diff = goal_pose.position.y - current_pose.position.y;
    if(dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
        heading = convertPlanarQuat2Phi(goal_pose.orientation); 
    }
    else {
        heading = atan2(y_diff, x_diff);
    }
    ROS_INFO("+++++++++++++++++++ current heading: %f", heading);

    dist = sqrt( x_diff*x_diff + y_diff*y_diff );
    ROS_INFO("+++++++++++++++++++ current distance: %f", dist);
}

// **************************************************************************************

int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 
    ros::NodeHandle nh;

    ROS_INFO("instantiating the timer_action_server: ");

    MobotActionServer as_object(nh); // create an instance of the class "MobotActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

