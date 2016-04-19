//find_plane_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// can select a patch; then computes a plane containing that patch, which is published on topic "planar_pts"
// illustrates use of PCL methods: computePointNormal(), transformPointCloud(), 
// pcl::PassThrough methods setInputCloud(), setFilterFieldName(), setFilterLimits, filter()
// pcl::io::loadPCDFile() 
// pcl::toROSMsg() for converting PCL pointcloud to ROS message
// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn March 2016

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <my_pcl_utils/my_pcl_utils.h>  //a local library with some utility fncs


using namespace std;
extern PclUtils *g_pcl_utils_ptr; 

//this fnc is defined in a separate module, find_indices_of_plane_from_patch.cpp
extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image

    vector<int> indices;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    // get the filename
    string fname = "/home/peng/ros_ws/src/student_code/PS9/my_pcd_images/coke_can.pcd";
    if (argc > 1) {
        fname = argv[1];
        ROS_INFO_STREAM("using fname passed in: " << fname);
    }
    else {
        ROS_INFO_STREAM("using default fname: " << fname);
    }
 
     if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) { //* load the file
        ROS_ERROR ("Couldn't read file \n");
        return (-1);
    }
    std::cout << "Loaded "
        << pclKinect_clr_ptr->width * pclKinect_clr_ptr->height
        << " data points from file "
        << fname
        << std::endl;

    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);

    sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too

    // cout << " select a patch of points to find corresponding plane..." << endl; //prompt user action
    // //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    // while (ros::ok()) {
    //     if (pclUtils.got_selected_points()) { //here if user selected a new patch of points
    //         pclUtils.reset_got_selected_points(); // reset for a future trigger
    //         pclUtils.get_copy_selected_points(selected_pts_cloud_ptr); //get a copy of the selected points
    //         cout << "got new patch with number of selected pts = " << selected_pts_cloud_ptr->points.size() << endl;

    //         //find pts coplanar w/ selected patch, using PCL methods in above-defined function
    //         //"indices" will get filled with indices of points that are approx co-planar with the selected patch
    //         // can extract indices from original cloud, or from voxel-filtered (down-sampled) cloud
    //         //find_indices_of_plane_from_patch(pclKinect_clr_ptr, selected_pts_cloud_ptr, indices);
    //         find_indices_of_plane_from_patch(downsampled_kinect_ptr, selected_pts_cloud_ptr, indices);
    //         pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud
    //         //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
    //         pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert to ros message for publication and display
    //     }
    //     pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
    //     pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
    //     pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
    //     ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
    //     ros::Duration(0.1).sleep();
    // }

    // variables
    ros::Publisher graspedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/graspedCloud", 1);
    ros::Publisher stoolCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stoolCloud", 1);
    ros::Publisher canCloud = nh.advertise<sensor_msgs::PointCloud2> ("/canCloud", 1);
    ros::Publisher planeCloud = nh.advertise<sensor_msgs::PointCloud2> ("/planeCloud", 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclGrasped_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclStool_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCan_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformedGrasped_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformedStool_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclTransformedCan_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    sensor_msgs::PointCloud2 grasped_cloud; //here are ROS-compatible messages
    sensor_msgs::PointCloud2 stool_cloud; //here are ROS-compatible messages
    sensor_msgs::PointCloud2 can_cloud; //here are ROS-compatible messages
    sensor_msgs::PointCloud2 plane_cloud; //here are ROS-compatible messages

    // grasp points by color
    // Eigen::Vector3i color;
    // color << 210, 200, 180;

    // pclUtils.graspPointsByColor(pclKinect_clr_ptr, pclGrasped_clr_ptr, color);
    // pcl::toROSMsg(*pclGrasped_clr_ptr, grasped_cloud); //convert from PCL cloud to ROS message this way

    // grasp points by space
    pclUtils.graspPointsBySpace(pclKinect_clr_ptr, pclGrasped_clr_ptr);
    pcl::toROSMsg(*pclGrasped_clr_ptr, grasped_cloud);

    // detect the stool
    pclUtils.detectStool(pclGrasped_clr_ptr, pclStool_clr_ptr);
    pcl::toROSMsg(*pclStool_clr_ptr, stool_cloud); //convert from PCL cloud to ROS message this way

    // compute the normal of the stool plane
    Eigen::Vector3f plane_normal;
    double plane_dist;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclStool_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pclUtils.downgradeCloud(pclStool_clr_ptr, pclStool_ptr);
    pclUtils.fit_points_to_plane(pclStool_ptr, plane_normal, plane_dist);
    cout << "corresponding evec (est plane normal): " <<plane_normal.transpose() <<endl;
    cout << "est plane distance from origin = " << plane_dist << endl;

    // compute the transformation matrix
    Eigen::Affine3f A = pclUtils.make_affine_from_plane_params(plane_normal, plane_dist);

    // get the transformed point cloud
    pclUtils.transform_kinect_cloud(A);
    // pclUtils.transform_cloud(A, pclGrasped_clr_ptr, pclTransformedGrasped_clr_ptr);
    pclUtils.transform_cloud(A, pclStool_clr_ptr, pclTransformedStool_clr_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedStool_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pclUtils.downgradeCloud(pclTransformedStool_clr_ptr, pclTransformedStool_ptr);
    Eigen::Vector3f xformed_stool_centroid = pclUtils.compute_centroid(pclTransformedStool_ptr);

    // compute the height of the camera
    double plane_height = -0.3;
    double z_eps = 0.01;

    pclUtils.find_coplanar_pts_z_height(plane_height, z_eps, indices);
    // pclUtils.copy_indexed_pts_to_output_cloud(indices, planeCloud);
    pclUtils.copy_cloud_xyzrgb_indices(pclKinect_clr_ptr, indices, plane_pts_ptr);
    pcl::toROSMsg(*plane_pts_ptr, ros_planar_cloud); //convert from PCL cloud to ROS message this way

    // pcl::PointXYZRGB camera_point, xformed_camera_point;
    // camera_point.getVector3fMap() <<< 0,0,0;
    // xformed_camera_point = A * camera_point.getVector3fMap();
    cout << "the height to the plane of stool is: " << 0 - xformed_stool_centroid[2];

    // detect the coke can
    pclUtils.detectCan(pclGrasped_clr_ptr, pclCan_clr_ptr);
    // pclUtils.transform_cloud(A, pclCan_clr_ptr, pclTransformedCan_clr_ptr);
    pcl::toROSMsg(*pclCan_clr_ptr, can_cloud); //convert from PCL cloud to ROS message this way

    // compute the coordinate of the can top
    Eigen::Vector3f can_top;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCan_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pclUtils.downgradeCloud(pclCan_clr_ptr, pclCan_ptr);
    can_top = pclUtils.compute_centroid(pclCan_ptr);
    cout<<"can_top coordinate: "<<can_top.transpose()<<endl;

    while (ros::ok()) {
        pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!

        graspedCloud.publish(grasped_cloud);
        stoolCloud.publish(stool_cloud);
        canCloud.publish(can_cloud);

        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }

    return 0;
}
