cmake_minimum_required(VERSION 2.8.3)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(cloud_image_tools)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")


find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  pcl_ros
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(PCL REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

catkin_package()

include_directories(src ${catkin_INCLUDE_DIRS})


add_executable(save_pcl_img_node src/save_pcl_img_node.cpp) 
target_link_libraries(save_pcl_img_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})  

add_executable(save_img_node src/save_img_node.cpp) 
target_link_libraries(save_img_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})  

add_executable(save_pcl_node src/save_pcl_node.cpp) 
target_link_libraries(save_pcl_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})  

add_executable(pcl2topic src/pcl2topic.cpp) 
target_link_libraries(pcl2topic ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})  

add_executable(cloud_filter src/cloud_filter.cpp) 
target_link_libraries(cloud_filter ${PCL_LIBRARIES} ${catkin_LIBRARIES})  

add_executable(pcd_tools src/pcd_tools.cpp) 
target_link_libraries(pcd_tools ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} stdc++fs)  

