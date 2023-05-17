#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/filesystem.hpp>
#include <vector>

#include "pcl_common.h"

bool try_load_pointcloud(const std::string &file_name, IPointCloud &cloud) {
  if (file_name.empty()) {
    ROS_ERROR_STREAM("Can't load pointcloud: no file name provided");
    return false;
  } else if (pcl::io::loadPCDFile(file_name, cloud) < 0) {
    ROS_ERROR_STREAM("Failed to parse pointcloud from file ('" << file_name
                                                               << "')");
    return false;
  }
  return true;
}

template <typename PointT>
bool voxel_grid_filter(const pcl::PointCloud<PointT> &cloud_src,
                       pcl::PointCloud<PointT> &cloud_dst,
                       float resample_size) {
  if (cloud_src.size() < 1) return false;

  pcl::VoxelGrid<PointT> resample_filter;
  resample_filter.setInputCloud(cloud_src.makeShared());
  resample_filter.setLeafSize(resample_size, resample_size, resample_size);
  resample_filter.filter(cloud_dst);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_tools");
  ros::NodeHandle nh;

  std::string folder_pcd_src, folder_pcd_dst;
  nh.getParam("/pcd_tools/folder_pcd_src", folder_pcd_src);
  nh.getParam("/pcd_tools/folder_pcd_dst", folder_pcd_dst);

  // look for images in input directory
  std::vector<std::string> vFilenames;
  boost::filesystem::directory_iterator itr;
  for (boost::filesystem::directory_iterator itr(folder_pcd_src);
       itr != boost::filesystem::directory_iterator(); ++itr) {
    if (!boost::filesystem::is_regular_file(itr->status())) {
      continue;
    }
    std::string filename = itr->path().filename().string();
    vFilenames.push_back(itr->path().string());
  }
  if (vFilenames.empty()) {
    std::cerr << "# ERROR: No chessboard images found." << std::endl;
    return 1;
  }
  std::sort(vFilenames.begin(), vFilenames.end());

  IPointCloud cloud_all;
  for (size_t i = 0; i < vFilenames.size(); ++i) {
    IPointCloud cloud_cur;
    if (try_load_pointcloud(vFilenames.at(i), cloud_cur)) {
      cloud_all += cloud_cur;
      ROS_INFO_STREAM("Success to add " << std::to_string(i) << " pcd file");
    } else {
      continue;
    }
  }
  if (cloud_all.size() > 0) {
    if (cloud_all.size() > 100000) {
      voxel_grid_filter(cloud_all, cloud_all, 0.1);
    }
    pcl::io::savePCDFile(folder_pcd_dst + "/pcd_all.pcd", cloud_all);
    ROS_WARN_STREAM("Success to add pointcloud, save to file ('"
                    << folder_pcd_dst + "pcd_all.pcd"
                    << "')");
  }
  return 0;
}