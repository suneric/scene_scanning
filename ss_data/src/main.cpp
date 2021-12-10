# include "process.h"
# include <ros/ros.h>
# include <string.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ssv3d;

PCLProcess processor;
PCLViewer viewer("3D Data Visualizer");

void DataCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 data;
  pcl_conversions::toPCL(*msg, data);
  WSPointCloudPtr cloud(new WSPointCloud);
  pcl::fromPCLPointCloud2(data,*cloud);
  WSPointCloudPtr res = processor.Registration(cloud);
  viewer.AddPointCloud(res);
  viewer.SpinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  std::string topic("/uav1/pointcloud");
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic,1,DataCallback);
  ros::spin();
  return 0;
}
