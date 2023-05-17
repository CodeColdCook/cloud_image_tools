//ms-iot.vscode-ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

#include <pcl_ros/point_cloud.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;
using namespace cv;

string image_topic, pcl_topic;
string IMG_SAVE_PATH, PCL_SAVE_PATH;
string img_save_name_headr, pcl_save_name_headr;
int frequence = 10, save_num = 0, delay_frame_count = 3;
bool if_save_img, if_save_pcl, if_show_img;

pcl::PointCloud<pcl::PointXYZ> point_cloud_buf;

void getPclParameters()
{
  cout << "getParameters pcl ..." << endl;
  if (
    !ros::param::get("pcl_topic", pcl_topic) 
    || !ros::param::get("PCL_SAVE_PATH", PCL_SAVE_PATH)
    || !ros::param::get("pcl_save_name_headr", pcl_save_name_headr)
    || !ros::param::get("if_save_pcl", if_save_pcl)
    || !ros::param::get("if_save_pcl", if_save_pcl)  )
  {
      cout << "Can not get the value of pcl save config or path or nameHeader..." << endl;
      exit(1);
  }
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pcl)
{
  save_num ++;
  cout << "pcl_callback " << save_num << endl;
  if(0 < (save_num % frequence) &&  (save_num % frequence) <= delay_frame_count)
  {
    pcl::PointCloud<pcl::PointXYZ> point_cloud_onece;
    pcl::fromROSMsg(*msg_pcl, point_cloud_onece);
    for(uint i=0; i< point_cloud_onece.size(); i++)
      point_cloud_buf.push_back(point_cloud_onece[i]);
  }

  if((save_num % frequence) == delay_frame_count)
  {
    if(point_cloud_buf.size() <10)
    {
      ROS_INFO("PCL EMPTY RETURN");
      // return;
    }
    ROS_INFO(" callback ...  ");
    if(if_save_pcl)
    {
      string save_path = PCL_SAVE_PATH + "/" +  pcl_save_name_headr + to_string(save_num) + ".pcd";
      pcl::io::savePCDFile(save_path, point_cloud_buf);
      ROS_INFO("SAVE successed");
    }
    point_cloud_buf.clear();
  }
  else
  {
    ROS_INFO("msg_pass ");
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_pcl_node");
  ros::NodeHandle n;
  ROS_INFO("start...");

  getPclParameters();
  ROS_INFO("wait for message");
  ros::Subscriber sub_pcl = n.subscribe(pcl_topic, 2000, pcl_callback);

  ros::spin();
  return EXIT_SUCCESS;
}