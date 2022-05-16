#include "process.h"
#include "display.h"
#include "octree.h"
#include "viewpoint.h"
#include "filter.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
# include <ros/ros.h>
# include <string.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace ssv3d;

OctoMap map(0.5);
PCLProcess processor;
PCLViewer viewer("3D Data Visualizer");
ros::Publisher map_pub;

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
      // viewer.AddCube(v.Center(), v.Length(), v.Id(), 0,0,1);
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

  std::vector<int> status;
  map.VoxelStatus(status);

  std_msgs::Int64MultiArray array;
  array.data.clear();
  for (size_t i = 0 ; i < status.size(); ++i)
    array.data.push_back(status[i]);

  map_pub.publish(array);
}

void PoseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  processor.AddViewPoint(msg->data);
}

double Distance(const WSPoint& pt1, const WSPoint& pt2)
{
  return sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y)+(pt1.z-pt2.z)*(pt1.z-pt2.z));
}

int main(int argc, char **argv)
{
  int display = 0;
  if (argc > 2)
    display = atoi(argv[1]);

  std::cout << "display type: " << display << '\n';

  ros::init(argc, argv, "data_process");
  ros::NodeHandle nh;
  std::string mapTopic("voxel_map");
  map_pub = nh.advertise<std_msgs::Int64MultiArray>(mapTopic,100);
  std::string vpTopic("/uav1/viewpoint");
  ros::Subscriber vpSub = nh.subscribe<std_msgs::Float64MultiArray>(vpTopic,1,PoseCallback);
  std::string ptTopic("/uav1/pointcloud");
  ros::Subscriber dataSub = nh.subscribe<sensor_msgs::PointCloud2>(ptTopic,1,DataCallback);

  if (display == 0)
  {
    std::cout << "dynamically visualize data" << '\n';

    Eigen::Vector3f vMin(-30.0,-30.0,0.0);
    Eigen::Vector3f vMax(30.0,30.0,10.0);

    std::cout << "initialize..." << std::endl;
    map.CreateMap(vMin, vMax);
  }
  else if (display == 1)
  {
    std::cout << "load data from files" << '\n';

    PCDisplay visualtool;
    std::string pt_file = "/home/yufeng/catkin_ws/src/scene_scanning/ss_data/data/airliner757.pcd";
    WSPointCloudPtr cloud = visualtool.LoadPointCloud(pt_file);
    viewer.AddPointCloud(cloud);

    std::vector<Eigen::Affine3f> cameras;
    std::string vp_file = "/home/yufeng/catkin_ws/src/scene_scanning/ss_data/data/best.txt";
    visualtool.LoadViewpoints(vp_file,cameras);

    double resolution = 0.5;
    PCLOctree octree(cloud,resolution,10);
    double voxelLen = octree.VoxelSideLength();
    PCLViewPoint viewCreator;

    double distance = 0.0;
    WSPoint startPt, endPt;
    std::vector<int> coveredVoxels;
    for (int i = 0; i < cameras.size(); ++i)
    {
      // display trajectory
      Eigen::Affine3f camera = cameras[i];
      if (i == 0) {
        endPt.x = camera.matrix()(0,3);
        endPt.y = camera.matrix()(1,3);
        endPt.z = camera.matrix()(2,3);
        startPt = endPt;
        viewer.AddCoordinateSystem(camera,i,2.0,0,true);
      } else {
        startPt = endPt;
        endPt.x = camera.matrix()(0,3);
        endPt.y = camera.matrix()(1,3);
        endPt.z = camera.matrix()(2,3);
        viewer.AddCoordinateSystem(camera,i,1.0,0,true);
      }

      viewer.AddLine(startPt,endPt,i,0.0,0.0,0.0);

      distance += Distance(startPt,endPt);

      // display covered voxels
      std::vector<int> indices;
      WSPointCloudPtr voxelCloud = viewCreator.CameraViewVoxels(octree, camera, indices);
      std::vector<WSPoint> vCenters;
      for (size_t j = 0; j < voxelCloud->points.size(); ++j)
      {
        WSPoint vCenter = voxelCloud->points[j];
        int idx = octree.VoxelIndex(vCenter);
        if (std::find(coveredVoxels.begin(), coveredVoxels.end(), idx) == coveredVoxels.end())
        {
          coveredVoxels.push_back(idx);
          vCenters.push_back(vCenter);
        }
      }

      for (size_t k = 0; k < vCenters.size(); ++k)
      {
        WSPoint c = vCenters[k];
        viewer.AddCube(c,voxelLen,i*100+k,0,0,1);
      }
    }

    std::cout << cameras.size() << " viewpoints with trajectory length of " << distance << " meters." << std::endl;
  }
  else if (display == 2)
  {
    double distance = 3.0;
    double resolution = 0.5;

    if (argc > 3)
    {
      distance = atof(argv[2]);
      resolution = atof(argv[3]);
    }
    std::cout << "generate viewpoints at " << distance << " with resolution " << resolution << '\n';

    // load configuration
    std::string config_file = "/home/yufeng/catkin_ws/src/scene_scanning/ss_data/data/config.txt";
    typedef std::vector<double> BBox;
    std::vector<std::pair<Eigen::Vector3f,BBox> > segments;
    std::ifstream config(config_file);
    std::string line;
    std::cout << "read config " << segments.size() << std::endl;

    // the viewpoint z limited in range of [height_min, height_max]
    double height_max = 0.0;
    double height_min = 0.0;
    while (std::getline(config, line))
    {
      std::cout << line << std::endl;
      std::stringstream linestream(line);
      std::string vpx,vpy,vpz,xmin,xmax,ymin,ymax,zmin,zmax;
      linestream >> vpx >> vpy >> vpz >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax >> height_min >> height_max;
      BBox box;
      box.push_back(std::stod(xmin));
      box.push_back(std::stod(xmax));
      box.push_back(std::stod(ymin));
      box.push_back(std::stod(ymax));
      box.push_back(std::stod(zmin));
      box.push_back(std::stod(zmax));
      Eigen::Vector3f refVP(std::stod(vpx),std::stod(vpy),std::stod(vpz));
      segments.push_back(std::make_pair(refVP,box));
    }
    config.close();

    // load point cloud
    PCDisplay visualtool;
    std::string pt_file = "/home/yufeng/catkin_ws/src/scene_scanning/ss_data/data/airliner757.pcd";
    WSPointCloudPtr srcCloud = visualtool.LoadPointCloud(pt_file);
    viewer.AddPointCloud(srcCloud);

    PCLFilter filter;
    PCLViewPoint viewCreator;
    PCLOctree octree(srcCloud,resolution,10);

    // generate a sub cloud for the bounding box and evaluate surface normals and cameras
    std::vector<Eigen::Affine3f> cameras;
    for (int i = 0; i < segments.size(); ++i)
    {
      Eigen::Vector3f refViewpoint = segments[i].first;
      BBox sbox = segments[i].second;
      WSPointCloudPtr segCloud = filter.FilterPCLPointInBBox(srcCloud,sbox,false);
      PCLOctree segOctree(segCloud,resolution,10);
      bool bSampling = false;
      viewCreator.GenerateCameraPositions(segOctree,distance,height_min,height_max,refViewpoint,bSampling,cameras);
    }

    // filter camera views
    std::vector<Eigen::Affine3f> cameras_valid;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
      Eigen::Affine3f camera = cameras[i];
      if (!viewCreator.FilterViewPoint(octree, camera))
        cameras_valid.push_back(camera);
    }

    // evaluate visible voxels
    std::map<int, std::vector<int> > viewVoxelMap;
    for (size_t i = 0; i < cameras_valid.size(); ++i)
    {
      std::vector<int> visibleVoxels;
      WSPointCloudPtr viewCloud = viewCreator.CameraViewVoxels(octree, cameras_valid[i],visibleVoxels);
      viewVoxelMap.insert(std::make_pair(static_cast<int>(i), visibleVoxels));
    }

    // display view points
    std::cout << "generate " << cameras_valid.size() << " valid viewpoints" << std::endl;
    for (size_t i = 0; i < cameras_valid.size(); ++i)
      viewer.AddCoordinateSystem(cameras_valid[i], i);

    // save viewpoints
    std::ostringstream oss1;
    oss1 << std::setprecision(2) << distance;
    std::string str_dist = oss1.str();
    std::ostringstream oss2;
    oss2 << std::setprecision(2) << resolution;
    std::string str_res = oss2.str();
    std::string fileName("/home/yufeng/catkin_ws/src/scene_scanning/ss_data/viewpoint/");
    fileName.append(str_dist).append("-").append(str_res).append(".txt");
    viewCreator.SaveToFile(fileName, cameras, viewVoxelMap);
    std::cout << "save to " << fileName << std::endl;
  }

  ros::Rate r(10);
  while(!viewer.IsStop())
  {
    ros::spinOnce();
    viewer.SpinOnce();
  }

  return 0;
}
