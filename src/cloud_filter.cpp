#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

using namespace std;

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pcl) {
  static int num = 0;
  pcl::PointCloud<pcl::PointXYZI> point_cloud_onece;
  pcl::fromROSMsg(*msg_pcl, point_cloud_onece);
  pcl::PointCloud<pcl::PointXYZI> cloud_dst;
  for (size_t i = 0; i < point_cloud_onece.size(); i++) {
    if (point_cloud_onece[i].x > 11 && point_cloud_onece[i].x < 16)
      if (point_cloud_onece[i].y > -1.7 && point_cloud_onece[i].y < 1.7)
        cloud_dst.push_back(point_cloud_onece[i]);
  }

  if (cloud_dst.size() < 2) return;
  string save_path = to_string(num) + ".pcd";
  pcl::io::savePCDFile(save_path, cloud_dst);
  num++;
  ROS_INFO("SAVE successed");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "save_pcl_node");
  ros::NodeHandle n;
  ROS_INFO("start...");

  ROS_INFO("wait for message");
  ros::Subscriber sub_pcl = n.subscribe("livox/lidar", 1, pcl_callback);

  ros::spin();
  return EXIT_SUCCESS;
}