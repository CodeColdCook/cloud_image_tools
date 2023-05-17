//ms-iot.vscode-ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <utility>  
#include <queue>

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

static queue<sensor_msgs::ImageConstPtr> img_buf;
static queue<sensor_msgs::PointCloud2ConstPtr> pcl_buf;
static mutex m_buf;

static string PCL_SAVE_PATH = "/home/cbreezy/002_everything_here/017_calibr_ws/src/lidar_camera_calib/calibr_b_data/pcd" ;
static string IMG_SAVE_PATH = "/home/cbreezy/002_everything_here/017_calibr_ws/src/lidar_camera_calib/calibr_b_data/img" ;
static int save_num = 0;


struct Measurement {
    sensor_msgs::Image img_msg;
    sensor_msgs::PointCloud2 pcl_msg;
};

void saveCallback(Measurement measurment)
{   
    if(save_num % 10 == 0){
        sensor_msgs::PointCloud2 msg_pcl = measurment.pcl_msg;
        sensor_msgs::Image msg_img = measurment.img_msg;

        // img
        Mat img;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_img);
        img = cv_ptr->image;
        string img_save_path = IMG_SAVE_PATH + "/" + to_string(save_num) + ".bmp";
        cv::imwrite(img_save_path,img);
        // pcl
//        pcl::PointCloud<pcl::PointXYZ> point_cloud_depth;
        pcl::PointCloud<pcl::PointXYZI> point_cloud_depth;
        pcl::fromROSMsg(msg_pcl, point_cloud_depth);
        string pcl_save_path = PCL_SAVE_PATH + "/" + to_string(save_num) + ".pcd";

        pcl::io::savePCDFile(pcl_save_path, point_cloud_depth);
        ROS_INFO("SAVE successed");
    }
    save_num++;
}

void img_callback(const sensor_msgs::ImageConstPtr &msg_img)
{
    // cout << "img_callback" << endl;
    m_buf.lock();
    if(img_buf.size() > 5)
        img_buf.pop();
    img_buf.push(msg_img);
    m_buf.unlock();
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pcl)
{
    // cout << "pcl_callback" << endl;
    m_buf.lock();
    if(pcl_buf.size() > 25)
        pcl_buf.pop();
    pcl_buf.push(msg_pcl);
    m_buf.unlock();
}

queue<Measurement> getMeasurements()
{
    queue<Measurement> measurements;
    // cout << "getMeasurements start " << endl;
    while(true)
    {
        if(img_buf.empty() || pcl_buf.empty() ){
            // cout << "something is empty " << endl;
            // cout << "img_buf: " << img_buf.size() << endl;
            // cout << "box_buf: " << box_buf.size() << endl;
            // cout << "pcl_buf: " << pcl_buf.size() << endl;
            return measurements;
        }
        // if(pcl_buf.front()->header.stamp < img_buf.front()->header.stamp){
        //     ROS_WARN("wait for lidar... ");
        //     pcl_buf.pop();
        //     return measurements;
        // }
        // if( (pcl_buf.front()->header.stamp.toSec() - img_buf.front()->header.stamp.toSec()) > 0.5 ){
        //     ROS_WARN("image delay for so long... ");
        //     img_buf.pop();
        //     box_buf.pop();
        //     return measurements;
        // }
        Measurement measurement;
        measurement.img_msg = *(img_buf.front());
        measurement.pcl_msg = *(pcl_buf.front());
        measurements.push(measurement);
        if(measurements.size() > 10)
            measurements.pop();
        // cout << "getMeasurements ok " << endl;
        return measurements;
        // usleep(5000);
    }
}

void process()
{
    cout << "process start " << endl;
    while(true)
    {
        queue<Measurement> measurements;
        // unique_lock<mutex> lk(m_buf);
        // con.wait(lk, [&]
        // {
        //     return (measurements = getMeasurements()).size() != 0;
        // });
        // lk.unlock();
        measurements = getMeasurements();
        while(measurements.size() != 0)
        {
            Measurement measurement;
            measurement = measurements.front();
            // cout << "processing the pcl and box " << endl;
            saveCallback(measurement);
            measurements.pop();
        }
        usleep(5000);
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_init_node");
    ros::NodeHandle n;

    ROS_INFO("... subscribe ... ");
    string img_topic, pcl_topic;
    if (!ros::param::get("img_topic", img_topic) 
            || !ros::param::get("pcl_topic", pcl_topic) )
    {
        cout << "Can not get the value of topics" << endl;
        exit(1);
    }
    cout << "img_topic: "<< img_topic << endl;
    cout << "pcl_topic: "<< pcl_topic << endl;
    ros::Subscriber sub_image = n.subscribe(img_topic, 1, img_callback);
    ros::Subscriber sub_pcl = n.subscribe(pcl_topic, 1, pcl_callback);
    thread measurement_process{process};
    ros::spin();
    return 0;
}
