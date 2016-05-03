#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_action_server/demoAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


//some tunable constants, global
const double g_move_speed = 0.5; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.5; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
const double PI = 3.1415926;

//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

int g_count = 0;
bool g_count_failure = false;


class ActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    actionlib::SimpleActionServer<my_action_server::demoAction> as_;

    ros::Publisher twist_commander;

    // here are some message types to communicate with our client(s)
    my_action_server::demoGoal goal_; // goal message, received from client
    my_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::demoFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    std::vector<geometry_msgs::PoseStamped> path_poses_;

    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    void do_halt();
    void do_move(double distance);
    void do_spin(double spin_ang);
    void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading);

public:
    ActionServer(); //define the body of the constructor outside of class definition

    ~ActionServer(void) {
    }
    // Action Interface
    void doInit();
    void executeCB();
    void preemptCB();
};


ActionServer::ActionServer() :
   as_(nh_, "my_lidar_mobot_action", false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ActionServer...");
    // do any other desired initializations here...specific to your implementation
    g_twist_commander = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ActionServer::executeCB, this));
    as_.registerPreemptCallback(boost::bind(&ActionServer::preemptCB, this));

    doInit();

    as_.start(); //start the server running
}


void ActionServer::executeCB() {
    ROS_INFO("in executeCB");
    goal_ = *(as_.acceptNewGoal());
    path_poses_ = goal_.path.poses;

    ROS_INFO("******** motion callback activated *********");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = path_poses_.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    yaw_current = 0;
    while(!path_poses_.empty()) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = path_poses_.front().pose; //get next pose from vector of poses
        path_poses_.erase(path_poses_.begin());
        
        // compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired, travel_distance, yaw_desired);        
        
        // ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        //yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        ROS_INFO("---------------------- spin angle: %f ----------------", spin_angle);
        do_spin(spin_angle); // carry out this incremental action

        // we will just assume that this action was successful--really should have sensor feedback here
        g_current_pose.position = pose_desired.position;
        g_current_pose.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
        yaw_current = yaw_desired;

        do_move(travel_distance);  // move forward 1m...just for illustration; SHOULD compute this from subgoal pose
    }

    //if here, then goal is still valid; provide some feedback
    feedback_.result = true; // populate feedback message with current countdown value
    as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal

    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}


void ActionServer::preemptCB() {
    ROS_INFO("Action Preempted");
    // set the action state to preempted
    as_.setPreempted();
    while(!path_poses_.empty()) {
        path_poses_.pop_back();
    }
}


void ActionServer::doInit() {
    
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;    
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0; 

    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;

    // g_twist_commander.publish(twist_cmd);
}


//signum function: strip off and return the sign of the argument
double ActionServer::sgn(double x) { 
    if(x>0.0) {return 1.0;}
    else if(x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double ActionServer::min_spin(double spin_angle) {
    if (spin_angle>M_PI) {
        spin_angle -= 2.0*M_PI;
    }
    if (spin_angle< -M_PI) {
        spin_angle += 2.0*M_PI;
    }
    return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double ActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion ActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void ActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
        g_twist_commander.publish(g_twist_cmd);
        timer += g_sample_dt;
        loop_timer.sleep(); 
    }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void ActionServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
        g_twist_commander.publish(g_twist_cmd);
        timer += g_sample_dt;
        loop_timer.sleep(); 
    }  
    do_halt();
}

void ActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
        g_twist_commander.publish(g_twist_cmd);
        loop_timer.sleep(); 
    }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void ActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {

    double x_diff = goal_pose.position.x - current_pose.position.x;
    double y_diff = goal_pose.position.y - current_pose.position.y;
    if(dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
        heading = convertPlanarQuat2Phi(goal_pose.orientation); 
    }
    else {
        heading = atan2(y_diff, x_diff);
    }
    ROS_INFO("+++++++++++++++++++ current heading: %f ++++++++++++++++++++++", heading);

    dist = sqrt( x_diff*x_diff + y_diff*y_diff );
    ROS_INFO("+++++++++++++++++++ current distance: %f ++++++++++++++++++++++", dist);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_server_node"); // name this node 

    ROS_INFO("====================== instantiating the my_action_server: ======================");

    ActionServer as_object; // create an instance of the class "ActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

