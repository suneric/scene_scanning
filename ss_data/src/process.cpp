#include <vector>
#include "process.h"
#include "filter.h"
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>


using namespace ssv3d;

PCLProcess::PCLProcess()
{
  m_cloudList.clear();
  m_viewpointList.clear();
  WSPointCloudPtr cloud(new WSPointCloud);
  m_cloud = cloud;
}

PCLProcess::~PCLProcess()
{
}

bool PCLProcess::RegisterPointCloud(OctoMap* map)
{
  size_t vpCount = m_viewpointList.size();
  size_t ptCount = m_cloudList.size();
  if (vpCount == 0 || ptCount == 0 || vpCount != ptCount)
    return false;
  // std::cout << "data count" << ptCount << '\n';
  // filter point cloud
  PCLFilter filter;
  WSPointCloudPtr cloud = m_cloudList[ptCount-1];
  cloud = filter.FilterPCLPoint(cloud, 0.01);
  // filter pt in z (0.2 m - 5 m) in local frame
  cloud = filter.FilterPassThrough(cloud,"z",0.2,5);

  // transform point cloud to global frame
  std::vector<double> vpMat = m_viewpointList[ptCount-1];
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      trans(i,j) = vpMat[i*4+j];

  WSPointCloudPtr tCloud (new WSPointCloud);
  pcl::transformPointCloud (*cloud, *tCloud, trans);
  // filter pt in z in global frame
  tCloud = filter.FilterPassThrough(tCloud,"z",0.05,30);
  *m_cloud += *tCloud;
  // add point cloud to map
  map->AddView(trans, tCloud);

  m_viewpointList.pop_back();
  m_cloudList.pop_back();

  return true;
}

void PCLProcess::AddViewPoint(const std::vector<double>& vp)
{
  m_viewpointList.push_back(vp);
}
void PCLProcess::AddPointCloud(const WSPointCloudPtr& cloud)
{
  m_cloudList.push_back(cloud);
}
