//ms-iot.vscode-ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>
#include <fstream>

#include <pcl_ros/point_cloud.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;
using namespace cv;

string image_topic, pcl_topic;
string IMG_SAVE_PATH, PCL_SAVE_PATH;
string img_save_name_headr, pcl_save_name_headr;
int frequence = 10, save_num = -1;
bool if_save_img, if_save_pcl, if_show_img;
bool if_undisort_img, if_save_undisort_img, if_show_undisort_img;
bool if_set_size_img, if_save_size_img, if_show_size_img;
bool if_undisort_to_resize;
int width_img_dst, height_img_dst;
Size size;
Mat cameraMatrix, distCoeffs;

void getImageParameters()
{
  cout << "getParameters img ..." << endl;
  if (
    !ros::param::get("image_topic", image_topic) 
    || !ros::param::get("IMG_SAVE_PATH", IMG_SAVE_PATH)
    || !ros::param::get("img_save_name_headr", img_save_name_headr)   
    || !ros::param::get("frequence", frequence)
    || !ros::param::get("if_save_img", if_save_img) 
    || !ros::param::get("if_show_img", if_show_img)
    || !ros::param::get("if_undisort_img", if_undisort_img)
    || !ros::param::get("if_save_undisort_img", if_save_undisort_img)
    || !ros::param::get("if_show_undisort_img", if_show_undisort_img)
    || !ros::param::get("if_set_size_img", if_set_size_img) ){
      cout << "Can not get the value of image save config or path or nameHeader..." << endl;
      exit(1);
  }
  if(if_set_size_img)
  {
    cout << "getParameters of set_size_img ..." << endl;
    if (
      !ros::param::get("width_img_dst", width_img_dst) 
      || !ros::param::get("height_img_dst", height_img_dst)
      || !ros::param::get("if_show_size_img", if_show_size_img)
      || !ros::param::get("if_save_size_img", if_save_size_img)
      || !ros::param::get("if_undisort_to_resize", if_undisort_to_resize) ){
      cout << "Can not get the value of width_img_dst or height_img_dst or save or show parameters ..." << endl;
      exit(1);  
    }
    if(width_img_dst < 50 || height_img_dst < 50){
      cout << "width_img_dst or height_img_dst is too small ..." << endl;
      cout << "width_img_dst: " << width_img_dst << endl;
      cout << "height_img_dst: " << height_img_dst << endl;
      exit(1); 
    }

  }
}

bool resizeImage()
{
}

// 获取相机内参
bool cameraInit(Mat& cameraMatrix_, Mat& distCoeffs_)
{
	string camera_config_path;
    if (!ros::param::get("camera_config_path", camera_config_path)){
        cout << "Can not get the value of camera_config_path" << endl;
        exit(1);
    }

    FileStorage fs(camera_config_path, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "camera_matrix file is wrong..." << endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;

    cout << "camera_matrix\n" << cameraMatrix_ << endl;
    cout << "\ndist coeffs\n" << distCoeffs_ << endl;

    return true;
}

// 图像去畸变
void undistort_img_tool(const Mat img_src, Mat& img_dst, const Mat cameraMatrix_, const Mat distCoeffs_)
{
    Mat map1, map2;
    Size imageSize;
    imageSize = img_src.size();
    initUndistortRectifyMap(cameraMatrix_, distCoeffs_, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, imageSize, 1, imageSize, 0),
    imageSize, CV_16SC2, map1, map2);
     
    remap(img_src, img_dst, map1, map2, INTER_LINEAR);
}

void img_callback(const sensor_msgs::ImageConstPtr &msg_img)
{
  save_num++;
  cout << "img_callback " << save_num << endl;
  if((save_num % frequence) == 0)
  {
    cout << "progress the image: " << save_num << endl;
    Mat colorImg;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img);
    colorImg = cv_ptr->image;
    // cv::cvtColor(colorImg, colorImg, cv::COLOR_BGR2RGB);

    if(!colorImg.empty())
    {
      // raw img
      if(if_save_img){
        string save_path = IMG_SAVE_PATH + "/" + img_save_name_headr + to_string(save_num) + ".jpg";
        cv::imwrite(save_path,colorImg);
      }
      if(if_show_img){
        imshow("raw_img",colorImg);
      }

      Mat undisortImg;
      // undisort the img
      if(if_undisort_img){
        undisortImg = colorImg.clone();
        undistort_img_tool(colorImg, undisortImg, cameraMatrix, distCoeffs);
        if(if_show_undisort_img)
          imshow("undisort_img",colorImg);
        if(if_save_undisort_img){
          string save_path = IMG_SAVE_PATH + "/" + img_save_name_headr + to_string(save_num) + ".jpg";
          cv::imwrite(save_path,undisortImg);
        }
      }
      
      // resize the img
      if(if_set_size_img){
        Mat resizeImg;
        if(if_undisort_to_resize) // frome undisort
          resize(undisortImg, resizeImg, size);
        else
          resize(colorImg, resizeImg, size);
        if(if_show_size_img)
          imshow("resize_img",resizeImg);
        if(if_save_size_img){
          string save_path = IMG_SAVE_PATH + "/" + img_save_name_headr + to_string(save_num) + ".jpg";
          cv::imwrite(save_path,resizeImg);
        }
      }
      
    }
    else
    {
      ROS_INFO("msg_pass ");
      return;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_image_node");
  ros::NodeHandle n;
  ROS_INFO("start...");

  getImageParameters();
  if(if_undisort_img){
    // cout << "if_undisort_img: " << if_undisort_img << endl;
    // cout << "if_show_undisort_img: " << if_show_undisort_img << endl;
    if(!cameraInit(cameraMatrix, distCoeffs))
      exit(1);
    if(if_show_undisort_img){
      cv::namedWindow("undisort_img");
      cv::startWindowThread();
    }
  }

  if(if_set_size_img){
    size.width = width_img_dst;
    size.height = height_img_dst;
    cout << "Set the image size to: " << size << endl;
    if(if_show_size_img){
      cv::namedWindow("resize_img");
      cv::startWindowThread();
    }
  }
    
  if(if_show_img){
    cv::namedWindow("raw_img");
    cv::startWindowThread();
  }

  ROS_INFO("wait for message");
  ros::Subscriber sub_image = n.subscribe(image_topic, 2000, img_callback);

  ros::spin();
  return EXIT_SUCCESS;
}