#include "octree.h"
#include "filter.h"
#include <math.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/geometry/planar_polygon.h>

using namespace ssv3d;

PCLOctree::PCLOctree(const WSPointCloudPtr& cloud, double resolution, int minVoxelPts)
{
  // define a octree search
  m_cloud = cloud;
  m_os = new OctreeSearch(resolution);
  m_os->setInputCloud(m_cloud);
  m_os->defineBoundingBox();
  m_os->addPointsFromInputCloud();

  // get occupied voxel centroids
  m_voxelMap.Clear();

  pcl::octree::OctreePointCloud<WSPoint>::AlignedPointTVector centroids;
  int nVoxel = m_os->getOccupiedVoxelCenters(centroids);
  double voxelSideLen = VoxelSideLength();
  for (int i = 0; i < nVoxel; ++i)
  {
    WSPoint center = centroids[i];
    std::vector<int> voxelIndices;
    if (m_os->voxelSearch(center,voxelIndices) && voxelIndices.size() >= minVoxelPts)
        m_voxelMap.AddVoxel(center,voxelSideLen);
  }

  std::vector<int> indices;
  int size = m_voxelMap.VoxelIndices(indices);
  WSPointCloudPtr voxelCloud(new WSPointCloud);
  voxelCloud->points.resize(size);
  for (int i = 0; i < size; ++i)
    voxelCloud->points[i] = m_voxelMap.GetVoxelCentroid(i);
  m_centroidCloud = voxelCloud;
}

PCLOctree::~PCLOctree()
{
  delete m_os;
}

PCLOctree::PCLOctree(const PCLOctree& tree)
{
  m_os = tree.m_os;
  m_cloud = tree.m_cloud;
  m_centroidCloud = tree.m_centroidCloud;
  m_voxelMap = tree.m_voxelMap;
}

PCLOctree& PCLOctree::operator=(const PCLOctree& tree)
{
  m_os = tree.m_os;
  m_cloud = tree.m_cloud;
  m_centroidCloud = tree.m_centroidCloud;
  m_voxelMap = tree.m_voxelMap;
  return *this;
}

WSPoint PCLOctree::VoxelCentroid(int index) const
{
  return m_voxelMap.GetVoxelCentroid(index);
}

double PCLOctree::VoxelSideLength() const
{
  return sqrt(m_os->getVoxelSquaredSideLen());
}

int PCLOctree::VoxelCount() const
{
  std::vector<int> vxIndices;
  return VoxelIndices(vxIndices);
}

int PCLOctree::VoxelIndices(std::vector<int>& indices) const
{
  return m_voxelMap.VoxelIndices(indices);
}

int PCLOctree::VoxelIndex(const WSPoint& centroid) const
{
  return m_voxelMap.VoxelIndex(centroid);
}

WSPointCloudPtr PCLOctree::VoxelCentroidCloud() const
{
  return m_centroidCloud;
}

namespace{
  bool InBoundingBox(const WSPoint& point, const std::vector<double>& bbox)
  {
    if (point.x < bbox[0] || point.x > bbox[1])
      return false;
    if (point.y < bbox[2] || point.y > bbox[3])
      return false;
    if (point.z < bbox[4] || point.z > bbox[5])
      return false;
    return true;
  }
}

WSPointCloudNormalPtr PCLOctree::VoxelAverageNormals_Sampling(
  const std::vector<int>& vxIndices,
  const Eigen::Vector3f& refViewpoint,
  WSPointCloudPtr& voxelCentroid) const
{
  if (nullptr == m_centroidCloud)
    return nullptr;

  std::cout << "evaluate surface normal with sampled points in the voxel \n";

  std::vector<Eigen::Vector3f> centroidVec;
  std::vector<Eigen::Vector3f> normalVec;

  // get all voxel cloud and evaluate normals
  PCLFilter filter;
  for (int i = 0; i < vxIndices.size(); ++i)
  {
    WSPoint point = VoxelCentroid(vxIndices[i]);
    std::vector<int> voxelIndices;
    if (!m_os->voxelSearch(point,voxelIndices) || voxelIndices.size() < 15)
      continue;

    // evaluate average normal of the voxel
    WSPointCloudPtr vCloud = filter.ExtractPoints(m_cloud, voxelIndices);
    WSPointCloudNormalPtr vNormal;
    vCloud = filter.SamplingSurfaceNormal(vCloud,15,0.1,vNormal);

    Eigen::Vector3f vec(0,0,0);
    int numCount = vNormal->points.size();
    for (size_t j = 0; j < numCount; ++j)
    {
      WSPoint point = vCloud->points[j];
      WSNormal normal = vNormal->points[j];
      Eigen::Vector3f nmVec = Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
      Eigen::Vector3f refVec = Eigen::Vector3f(refViewpoint.x()-point.x,refViewpoint.y()-point.y,refViewpoint.z()-point.z);
      if (nmVec.dot(refVec) < 0.0) // normal angle > M_PI/2
        nmVec = -nmVec;

      vec += Eigen::Vector3f(nmVec.x(), nmVec.y(), nmVec.z());
    }
    Eigen::Vector3f nm = (vec/numCount).normalized();

    // check visibility
    Eigen::Vector3f centerPt = Eigen::Vector3f(point.x, point.y,point.z);
    int nRes = IntersectedOccupiedVoxels(refViewpoint, centerPt);
    if (nRes > 1)
      continue;

    centroidVec.push_back(centerPt);
    normalVec.push_back(nm);
  }

  int vCount = centroidVec.size();
  WSPointCloudPtr centroids(new WSPointCloud);
  centroids->points.resize(vCount);
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  normals->points.resize(vCount);
  for (size_t i = 0; i < vCount; ++i)
  {
    Eigen::Vector3f pt = centroidVec[i];
    centroids->points[i].x = pt.x();
    centroids->points[i].y = pt.y();
    centroids->points[i].z = pt.z();
    Eigen::Vector3f nm = normalVec[i];
    normals->points[i].normal_x = nm.x();
    normals->points[i].normal_y = nm.y();
    normals->points[i].normal_z = nm.z();
  }
  voxelCentroid = centroids;
  return normals;
}

WSPointCloudNormalPtr PCLOctree::VoxelAverageNormals_All(
  const std::vector<int>& vxIndices,
  const Eigen::Vector3f& refViewpoint,
  WSPointCloudPtr& voxelCentroid) const
{
  if (nullptr == m_centroidCloud)
    return nullptr;

  std::cout << "evaluate surface normal with all points in the voxel \n";

  std::vector<Eigen::Vector3f> centroidVec;
  std::vector<Eigen::Vector3f> normalVec;

  PCLFilter filter;
  for (int i = 0; i < vxIndices.size(); ++i)
  {
    WSPoint point = VoxelCentroid(vxIndices[i]);
    std::vector<int> voxelIndices;
    if (!m_os->voxelSearch(point,voxelIndices))
    {
      std::cout << "no voxel leaf node exist at [ " << point.x << ", " << point.y << ", " << point.z << "]" << std::endl;
      continue;
    }

    // evaluate average normal of the voxel
    WSPointCloudPtr vCloud = filter.ExtractPoints(m_cloud, voxelIndices);
    WSPointCloudNormalPtr vNormal(new WSPointCloudNormal);
    pcl::NormalEstimation<WSPoint, WSNormal> ne;
    pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(vCloud);
    ne.setViewPoint(refViewpoint.x(),refViewpoint.y(),refViewpoint.z());
    //ne.setKSearch(10);
    double rad = 0.5*VoxelSideLength();
    ne.setRadiusSearch(rad);
    ne.compute(*vNormal);

    int numCount = vNormal->points.size();
    if (numCount == 0)
    {
      std::cout << "no normal evaluated " << std::endl;
      continue;
    }

    // find average normal for the voxel
    Eigen::Vector3f vec(0,0,0);
    int count = 0;
    for (size_t j = 0; j < numCount; ++j)
    {
      WSNormal normal = vNormal->points[j];
      if (isnan(normal.normal_x) || isnan(normal.normal_y) || isnan(normal.normal_z))
        continue;

      vec += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
      count++;
    }
    Eigen::Vector3f nm = (vec/count).normalized();
    // std::cout << "Normal (" << nm.x() << ", " << nm.y() << ", " << nm.z() << ")" << std::endl;

    // check visibility, if the voxel is blocked by others
    Eigen::Vector3f centerPt = Eigen::Vector3f(point.x, point.y,point.z);
    Eigen::Vector3f refPt = centerPt + 3.0*nm;
    int nRes = IntersectedOccupiedVoxels(refPt, centerPt);
    if (nRes > 1)
      continue;

    centroidVec.push_back(centerPt);
    normalVec.push_back(nm);
  }

  int vCount = centroidVec.size();
  WSPointCloudPtr centroids(new WSPointCloud);
  centroids->points.resize(vCount);
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  normals->points.resize(vCount);
  for (size_t i = 0; i < vCount; ++i)
  {
    Eigen::Vector3f pt = centroidVec[i];
    centroids->points[i].x = pt.x();
    centroids->points[i].y = pt.y();
    centroids->points[i].z = pt.z();
    Eigen::Vector3f nm = normalVec[i];
    normals->points[i].normal_x = nm.x();
    normals->points[i].normal_y = nm.y();
    normals->points[i].normal_z = nm.z();
  }

  voxelCentroid = centroids;
  return normals;
}

int PCLOctree::IntersectedOccupiedVoxels(const Eigen::Vector3f& origin, const Eigen::Vector3f& end) const
{
  // get a point vector of centers of voxels intersected by a line segment
  // return an approximation of the actual intersected voxels by walking along the line with small steps.
  pcl::octree::OctreePointCloud<WSPoint>::AlignedPointTVector tmpCentroids;
  int nInterSectVoxel = m_os->getApproxIntersectedVoxelCentersBySegment(origin,end,tmpCentroids,0.1);
  int count = 0;
  for (size_t j = 0; j < tmpCentroids.size(); ++j)
  {
    // Check if voxel at given point exist.
    bool bOccupied = m_os->isVoxelOccupiedAtPoint(tmpCentroids[j]);
    if (bOccupied)
      count++;
  }
  return count;
}

int PCLOctree::BoxSearch(const Eigen::Vector3f& minPt,
              const Eigen::Vector3f& maxPt,
              std::vector<int>& indices) const
{
  // get all points in a bounding box
  return m_os->boxSearch(minPt, maxPt, indices);
}

int PCLOctree::VoxelIndicesInBox(const std::vector<double>& bbox, std::vector<int>& indices) const
{
  // return centroid of voxels in the bounding box
  for (int i = 0; i < VoxelCount(); ++i)
  {
    WSPoint point = VoxelCentroid(i);
    if (InBoundingBox(point, bbox))
      indices.push_back(i);
  }
  return indices.size();
}
