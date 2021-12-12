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
  processor.AddPointCloud(cloud);
  bool res = processor.Registration();
  if (res)
  {
    viewer.AddPointCloud(processor.PointCloud());
  }
}

void PoseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  processor.AddViewPoint(msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  std::string vpTopic("/uav1/viewpoint");
  ros::Subscriber vpSub = nh.subscribe<std_msgs::Float64MultiArray>(vpTopic,1,PoseCallback);
  std::string ptTopic("/uav1/pointcloud");
  ros::Subscriber dataSub = nh.subscribe<sensor_msgs::PointCloud2>(ptTopic,1,DataCallback);
  ros::Rate r(10);
  while(!viewer.IsStop())
  {
    ros::spinOnce();
    viewer.SpinOnce();
  }

  return 0;
}
