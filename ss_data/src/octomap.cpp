#include <set>
#include <map>
#include "octomap.h"
#include "filter.h"
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/frustum_culling.h>

using namespace ssv3d;

OctoMap::OctoMap(double resolution)
{
  m_resolution = resolution;
  m_centerCloud = nullptr;
  m_points.clear();
  m_voxels.clear();
}

OctoMap::~OctoMap()
{
}

void OctoMap::CreateMap(const Eigen::Vector3f& min, const Eigen::Vector3f& max)
{
  m_centerCloud = WSPointCloudPtr(new WSPointCloud);
  double len = m_resolution;
  int id = 0;
  for (double z = min.z(); z <= max.x(); z+=len)
  {
    for(double y = min.y(); y <= max.y(); y+=len)
    {
      for (double x = min.x(); x <= max.x(); x+=len)
      {
        Voxel3d v(id,m_resolution,Eigen::Vector3f(x,y,z),Eigen::Vector3f(x+len,y+len,z+len));
        m_voxels.push_back(v);
        m_centerCloud->push_back(v.Center());
        id++;
      }
    }
  }
}

void OctoMap::AddView(const Eigen::Matrix4f& viewPoint, const WSPointCloudPtr& cloud)
{
  std::vector<int> voxelIndices;
  ViewVoxels(viewPoint, voxelIndices);
  if (voxelIndices.empty())
    return;

  std::vector<int> occupiedIndices;
  AddPointToVoxel(voxelIndices, cloud, occupiedIndices);
  AddStatusToVoxel(viewPoint, voxelIndices);
  if (occupiedIndices.empty())
    return;

  AddCentroidToVoxel(occupiedIndices);
  AddNormalToVoxel(occupiedIndices, viewPoint);
}

void OctoMap::AddPointToVoxel(const std::vector<int>& voxelIndices, const WSPointCloudPtr& cloud, std::vector<int>& occupiedIndices)
{
  // std::cout << "Add Points" << std::endl;
  PCLFilter filter;
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];

    double len = m_resolution;
    WSPoint center = m_voxels[vxIdx].Center();
    std::vector<double> bbox;
    bbox.push_back(center.x-0.5*len); // min x
    bbox.push_back(center.y-0.5*len); // min y
    bbox.push_back(center.z-0.5*len); // min z
    bbox.push_back(center.x+0.5*len); // max x
    bbox.push_back(center.y+0.5*len); // max y
    bbox.push_back(center.z+0.5*len); // max z
    WSPointCloudPtr vCloud = filter.FilterPCLPointInBBox(cloud, bbox);
    if (vCloud->points.size() < 10)
      continue;

    for (size_t j = 0; j < vCloud->points.size(); ++j)
    {
      WSPoint pt = vCloud->at(j);
      m_points.push_back(pt);
      m_voxels[vxIdx].AddPointIndex(m_points.size()-1);
    }
    occupiedIndices.push_back(vxIdx);
    // std::cout << i << " points number in voxel " << vxIdx << ":" << vCloud->points.size() << std::endl;
  }
  std::cout << "cloud points count " << m_points.size() << std::endl;
}

void OctoMap::AddCentroidToVoxel(const std::vector<int>& voxelIndices)
{
  // std::cout << "Add Centroid" << std::endl;
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];

    pcl::octree::OctreePointCloudVoxelCentroidContainer<WSPoint> vc;
    std::vector<int> ptIndices;
    m_voxels[vxIdx].PointIndices(ptIndices);
    for (size_t j = 0; j < ptIndices.size(); ++j)
    {
      WSPoint pt = m_points[ptIndices[j]];
      vc.addPoint(pt);
    }
    WSPoint centroid;
    vc.getCentroid(centroid);
    m_voxels[vxIdx].AddCentroid(centroid);
    // std::cout << i << " centroid of voxel " << vxIdx << ":"<< centroid.x <<" " << centroid.y << " " << centroid.z << std::endl;
  }
}
void OctoMap::AddNormalToVoxel(const std::vector<int>& voxelIndices, const Eigen::Matrix4f& viewPoint)
{
  // std::cout << "Add Normal" << std::endl;
  Eigen::Vector3f refNormal = (viewPoint*Eigen::Vector4f(0,0,1,1)).head<3>().normalized();
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];
    WSPointCloudPtr vCloud(new WSPointCloud);
    std::vector<int> ptIndices;
    m_voxels[vxIdx].PointIndices(ptIndices);
    for (size_t j = 0; j < ptIndices.size(); ++j)
    {
      WSPoint pt = m_points[ptIndices[j]];
      vCloud->push_back(pt);
    }

    // evaluate normal
    WSPointCloudNormalPtr normals(new WSPointCloudNormal);
    pcl::NormalEstimation<WSPoint, WSNormal> ne;
    pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(vCloud);
    ne.setRadiusSearch(0.1); // use all neighbors in 10 cm
    ne.compute(*normals);

    WSNormal normal;
    Eigen::Vector3f vec(0,0,0);
    int num = normals->points.size();
    if (num == 0)
      continue;

    for (size_t k = 0; k < num; ++k)
    {
      WSNormal n = normals->points[k];
      vec += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
    }
    Eigen::Vector3f nm = (vec/num).normalized();
    if (nm.dot(refNormal) < 0.0) // normal angle > M_PI/2
      nm = -nm;
    normal.normal_x = nm.x();
    normal.normal_y = nm.y();
    normal.normal_z = nm.z();
    m_voxels[vxIdx].AddNormal(normal);
    // std::cout << i << " normal of voxel " << vxIdx << ":" << normal.normal_x <<" " << normal.normal_y << " " << normal.normal_z << std::endl;
  }
}

void OctoMap::AddStatusToVoxel(const Eigen::Matrix4f& viewPoint, const std::vector<int>& voxelIndices)
{
  // check status: 0: unknown, 1: free, 2: occupied, 3: occluded
  Eigen::Vector3f origin(viewPoint(0,3),viewPoint(1,3),viewPoint(2,3));
  // std::cout << "Add Status" << std::endl;
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];
    // std::cout << "update voxel status " << i << " " << vxIdx << " " << voxelIndices.size() << std::endl;
    // occuplied voxels status
    if (!m_voxels[vxIdx].Empty())
    {
      m_voxels[vxIdx].SetStatus(2);
      continue;
    }

    WSPoint center = m_voxels[vxIdx].Center();
    Eigen::Vector3f target(center.x,center.y,center.z);
    // unoccupied voxel status
    std::vector<int> intersect;
    RayTrace(origin, target, voxelIndices, intersect);
    if (intersect.size() == 0)
    {
      m_voxels[vxIdx].SetStatus(0); // unknown
    }
    else if (intersect.size() == 1)
    {
      m_voxels[vxIdx].SetStatus(1); // free
    }
    else
    {
      // if previuse voxels are occupied
      int occupiedCount = 0;
      for (size_t j = 0; j < intersect.size()-1; ++j)
      {
        if (!m_voxels[intersect[j]].Empty())
          occupiedCount++;
      }
      if (occupiedCount > 0)
        m_voxels[vxIdx].SetStatus(3);
      else
        m_voxels[vxIdx].SetStatus(1); // free
    }
  }
}

void OctoMap::RayTrace(const Eigen::Vector3f& origin, const Eigen::Vector3f& target, const std::vector<int>& voxelIndices, std::vector<int>& intersectedVoxels)
{
  // std::cout << "Ray Trace" << std::endl;
  Eigen::Vector3f normal = target-origin;
  double distance = normal.norm();
  Eigen::Vector3f direction = normal.normalized();
  // std::cout << "start " << origin.x() << " " << origin.y() << " " << origin.z() << std::endl;
  // std::cout << "end " << target.x() << " " << target.y() << " " << target.z() << std::endl;
  // std::cout << "diretion " << direction.x() << " " << direction.y() << " " << direction.z() << std::endl;
  // std::cout << "distance " << distance << std::endl;

  // make a copy
  std::vector<int> checkIndices;
  for (size_t i = 0; i < voxelIndices.size(); ++i)
    checkIndices.push_back(voxelIndices[i]);

  intersectedVoxels.clear();
  double s = 0;
  while (s <= distance+0.01)
  {
    // std::cout << "voxels to be checked " << s << " " << checkIndices.size() << std::endl;
    int checkVoxel = -1;
    Eigen::Vector3f check = origin+s*direction;
    if (CheckIntersection(check, checkIndices, checkVoxel))
    {
      std::vector<int>::iterator it = std::find(intersectedVoxels.begin(), intersectedVoxels.end(), checkVoxel);
      if (it == intersectedVoxels.end())
        intersectedVoxels.push_back(checkVoxel);
    }
    s += 0.05*distance;
  }
  // std::cout << "found intersection " << intersectedVoxels.size() << std::endl;
}

bool OctoMap::CheckIntersection(const Eigen::Vector3f& check, std::vector<int>& voxelIndices, int& intersectedVoxel)
{
  WSPoint pt;
  pt.x = check.x();
  pt.y = check.y();
  pt.z = check.z();
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];
    if (m_voxels[vxIdx].InVoxel(pt))
    {
      intersectedVoxel = vxIdx;
      voxelIndices.erase(voxelIndices.begin()+i);
      return true;
    }
  }
  return false;
}

void OctoMap::ViewVoxels(const Eigen::Matrix4f& viewPoint, std::vector<int>& voxelIndices)
{
  WSPointCloudPtr viewCloudCenter(new WSPointCloud);
  if (FrustumCulling(viewPoint, viewCloudCenter))
  {
    for (size_t i = 0; i < viewCloudCenter->points.size(); ++i)
    {
        int vxIdx = -1;
        WSPoint center = viewCloudCenter->at(i);
        if(VoxelIndex(center,vxIdx));
          voxelIndices.push_back(vxIdx);
    }
  }
  std::cout << "view voxels count " << voxelIndices.size() << std::endl;
}

// Find voxel by the center point
bool OctoMap::VoxelIndex(const WSPoint& pt, int& voxelIndex)
{
  for (size_t i = 0; i < m_voxels.size(); ++i)
  {
    if (m_voxels[i].InVoxel(pt))
    {
      voxelIndex = i;
      return true;
    }
  }
  return false;
}

bool OctoMap::FrustumCulling(const Eigen::Matrix4f& view, WSPointCloudPtr viewCloud)
{
  if (nullptr == m_centerCloud)
    return false;

  Eigen::Matrix4f poseNew = CameraPoseTransform(view);
  // frustum culling
  pcl::FrustumCulling<WSPoint> fc;
  fc.setInputCloud(m_centerCloud);
  fc.setCameraPose(poseNew);
  fc.setHorizontalFOV(60); // degree
  fc.setVerticalFOV(40); // degree
  fc.setNearPlaneDistance(0.2);
  fc.setFarPlaneDistance(5.0);
  fc.filter(*viewCloud);
  return true;
}

// This assume a coordinate system where X is forward, Y is up and Z is right.
// to convert from the traditional camera coordinate syste (X is right, Y is down, Z forward)
// [0,0,1,0
//  0,-1,0,0
//  1,0,0,0
//  0,0,0,1]
Eigen::Matrix4f OctoMap::CameraPoseTransform(const Eigen::Matrix4f& mat)
{
  Eigen::Matrix4f cam2rot;
  cam2rot(0,0) = 0;
  cam2rot(0,1) = 0;
  cam2rot(0,2) = 1;
  cam2rot(0,3) = 0;
  cam2rot(1,0) = 0;
  cam2rot(1,1) = -1;
  cam2rot(1,2) = 0;
  cam2rot(1,3) = 0;
  cam2rot(2,0) = 1;
  cam2rot(2,1) = 0;
  cam2rot(2,2) = 0;
  cam2rot(2,3) = 0;
  cam2rot(3,0) = 0;
  cam2rot(3,1) = 0;
  cam2rot(3,2) = 0;
  cam2rot(3,3) = 1;
  return mat*cam2rot;
}

void OctoMap::OccupiedVoxels(std::vector<Voxel3d>& voxels)
{
  for (int i = 0; i < m_voxels.size(); ++i)
  {
    if (m_voxels[i].Status() == 2)
      voxels.push_back(m_voxels[i]);
  }
  std::cout << "Occupied Voxels " << voxels.size() << std::endl;
}

void OctoMap::FreeVoxels(std::vector<Voxel3d>& voxels)
{
  for (int i = 0; i < m_voxels.size(); ++i)
  {
    if (m_voxels[i].Status() == 1)
      voxels.push_back(m_voxels[i]);
  }
  std::cout << "Free Voxels " << voxels.size() << std::endl;
}

void OctoMap::OccludedVoxels(std::vector<Voxel3d>& voxels)
{
  for (int i = 0; i < m_voxels.size(); ++i)
  {
    if (m_voxels[i].Status() == 3)
      voxels.push_back(m_voxels[i]);
  }
  std::cout << "Occluded Voxels " << voxels.size() << std::endl;
}

WSPointCloudPtr OctoMap::PointCloud() const
{
  WSPointCloudPtr cloud(new WSPointCloud);
  for (size_t i = 0; i < m_points.size(); ++i)
    cloud->push_back(m_points[i]);
  return cloud;
}

void OctoMap::VoxelStatus(std::vector<int>& map) const
{
  for (int i = 0; i < m_voxels.size(); ++i)
    map.push_back(m_voxels[i].Status());
}
