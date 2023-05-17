#include <iostream>
#include <string>
#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
using namespace std; 

string pcd_file_path = "/home/cbreezy/001_utils/000_img_pcl_tool_ws/files/kalibr_GY_livox_box_1/pcl/kalibr_pcl_3.pcd";
int main(int argc,char ** argv)
{
  ROS_INFO("Log In Success!!!!!!!");
  ros::init (argc, argv, "pcdp"); 
  ros::NodeHandle n; 

  if (!ros::param::get("pcd_file_path", pcd_file_path))
  {
      cout << "Can not get the value of pcl save config or path or nameHeader..." << endl;
      exit(1);
  }

  ros::Publisher pcl_pub;
  pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("pcdp_output", 1);    
  pcl::PointCloud<pcl::PointXYZ> testcloud; //point cloud msg  
  sensor_msgs::PointCloud2 output; //PointCloud2 msg

  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, testcloud);
  // testcloud.toROSMsg(testcloud, output);
  ros::Rate rate(1);
  while (ros::ok())
  {
    pcl::toROSMsg(testcloud,output); //point cloud msg -> ROS msg
    output.header.frame_id="pcdp";
    pcl_pub.publish(output); //publish
    ros::spinOnce();
    rate.sleep();
  }
  // ros::spin();
    return 0;
}
