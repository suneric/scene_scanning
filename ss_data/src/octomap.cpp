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
  m_occupiedVoxelIndices.clear();
}

OctoMap::~OctoMap()
{
}

int OctoMap::VoxelIndex(const WSPoint& pt)
{
  for (size_t i = 0; i < m_voxels.size(); ++i)
  {
    if (m_voxels[i].InVoxel(pt))
      return i;
  }
  return -1;
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

void OctoMap::AddView(const Eigen::Matrix4f& viewpoint, const WSPointCloudPtr& cloud)
{
  std::cout << "Add View " << std::endl;
  // check the voxels (center) in the view
  Eigen::Vector3f viewNormal = (viewpoint*Eigen::Vector4f(0,0,1,0)).head<3>().normalized();

  std::vector<int> voxelIndices;
  WSPointCloudPtr viewCloud(new WSPointCloud);
  if (FrustumCulling(viewpoint, viewCloud))
  {
    for (size_t i = 0; i < viewCloud->points.size(); ++i)
    {
        int vxIdx = VoxelIndex(viewCloud->at(i));
        voxelIndices.push_back(vxIdx);
    }
  }

  std::cout << "View Voxels " << voxelIndices.size() << std::endl;

  PCLFilter filter;
  for (size_t i = 0; i < voxelIndices.size(); ++i)
  {
    int vxIdx = voxelIndices[i];
    std::cout << "update voxel " << vxIdx << std::endl;
    // fill voxels
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
    std::cout << "points number " << vCloud->points.size() << std::endl;
    if (vCloud->points.size() < 5)
    {
      // check status: free, unknown, occuplied, occluded
      continue;
    }

    m_occupiedVoxelIndices.push_back(vxIdx);

    // evaluate centroids
    WSPoint centroid;
    pcl::octree::OctreePointCloudVoxelCentroidContainer<WSPoint> vc;
    for (size_t j = 0; j < vCloud->points.size(); ++j)
    {
      WSPoint pt = vCloud->at(j);
      vc.addPoint(pt);
      m_points.push_back(pt);
      m_voxels[vxIdx].AddPointIndex(m_points.size()-1);
    }
    vc.getCentroid(centroid);
    std::cout << "centroid " << centroid.x <<" " << centroid.y << " " << centroid.z << std::endl;
    m_voxels[vxIdx].AddCentroid(centroid);

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
    for (size_t k = 0; k < num; ++k)
    {
      WSNormal n = normals->points[k];
      vec += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
    }
    Eigen::Vector3f nm = (vec/num).normalized();
    if (nm.dot(viewNormal) < 0.0) // normal angle > M_PI/2
      nm = -nm;
    normal.normal_x = nm.x();
    normal.normal_y = nm.y();
    normal.normal_z = nm.z();
    std::cout << "normal " << normal.normal_x <<" " << normal.normal_y << " " << normal.normal_z << std::endl;
    m_voxels[vxIdx].AddNormal(normal);
  }
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
  std::cout << "Occupied Voxels " << m_occupiedVoxelIndices.size() << std::endl;
  for (int i = 0; i < m_occupiedVoxelIndices.size(); ++i)
  {
    voxels.push_back(m_voxels[m_occupiedVoxelIndices[i]]);
  }
}

void OctoMap::FreeVoxels(std::vector<Voxel3d>& voxels)
{

}

void OctoMap::UnknownVoxels(std::vector<Voxel3d>& voxels)
{
  for (auto it = m_voxels.begin(); it != m_voxels.end(); ++it)
    voxels.push_back(*it);
}

WSPointCloudPtr OctoMap::PointCloud() const
{
  WSPointCloudPtr cloud(new WSPointCloud);
  for (size_t i = 0; i < m_points.size(); ++i)
    cloud->push_back(m_points[i]);
  return cloud;
}
