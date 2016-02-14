//my ROS service:
// run this as: rosrun my_ROS_service my_ROS_service
// in another window, tickle it manually with (e.g.): 
//    rosservice call lookup_by_name 'Ted'


#include <ros/ros.h>
#include <my_ros_service/MyServiceMsg.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(my_ros_service::MyServiceMsgRequest& request, my_ros_service::MyServiceMsgResponse& response)
{
    ROS_INFO("callback activated");
    string in_name(request.name); //let's convert this to a C++-class string, so can use member funcs
    //cout<<"in_name:"<<in_name<<endl;
    response.on_the_list=false;
    
    // here is a dumb way to access a stupid database...
    // hey: this example is about services, not databases!
    if (in_name.compare("Bob")==0)
 {
        ROS_INFO("asked about Bob");
        response.age = 32;
        response.good_guy=false;
        response.on_the_list=true;
        response.nickname="BobTheTerrible";
    } 
     if (in_name.compare("Ted")==0)
 {
        ROS_INFO("asked about Ted");
        response.age = 21;
        response.good_guy=true;
        response.on_the_list=true;
        response.nickname="Ted the Benevolent";
    }    
    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_ros_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("lookup_by_name", callback);
  ROS_INFO("Ready to look up names.");
  ros::spin();

  return 0;
}
