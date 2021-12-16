# include "process.h"
# include <ros/ros.h>
# include <string.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ssv3d;

OctoMap map(0.2);
PCLProcess processor;
PCLViewer viewer("3D Data Visualizer");

void DataCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // std::cout << "convert ros point cloud to pcl point cloud..." << std::endl;
  pcl::PCLPointCloud2 data;
  pcl_conversions::toPCL(*msg, data);
  WSPointCloudPtr cloud(new WSPointCloud);
  pcl::fromPCLPointCloud2(data,*cloud);

  // std::cout << "add point cloud..." << std::endl;
  processor.AddPointCloud(cloud);

  // std::cout << "register point cloud..." << std::endl;
  bool res = processor.RegisterPointCloud(&map);
  if (res)
  {
    // std::cout << "update point cloud..." << std::endl;
    std::vector<Voxel3d> occupiedVoxels, freeVoxels, occludedVoxels;
    map.OccludedVoxels(occludedVoxels);
    map.OccupiedVoxels(occupiedVoxels);
    map.FreeVoxels(freeVoxels);
    std::cout << "add point cloud to view..." << std::endl;
    viewer.AddPointCloud(map.PointCloud());
    for (size_t i=0; i < occupiedVoxels.size(); ++i)
    {
      Voxel3d v = occupiedVoxels[i];
      viewer.AddCube(v.Center(), v.Length(), v.Id(), 0,0,1);
      // viewer.AddArrow(v.Centroid(),v.Normal(),0.1,v.Id(),0.1,0,1,0);
    }
    // for (size_t i=0; i < freeVoxels.size(); ++i)
    // {
    //   Voxel3d v = freeVoxels[i];
    //   viewer.AddCube(v.Center(), v.Length(), v.Id(), 0,1,0);
    // }
    // for (size_t i=0; i < occludedVoxels.size(); ++i)
    // {
    //   Voxel3d v = occludedVoxels[i];
    //   viewer.AddCube(v.Center(), v.Length(), v.Id(), 1,0,0);
    // }
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

  Eigen::Vector3f vMin(-6.0,-6.0,0.0);
  Eigen::Vector3f vMax(3.0,6.0,5.0);

  std::cout << "initialize..." << std::endl;
  map.CreateMap(vMin, vMax);

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
