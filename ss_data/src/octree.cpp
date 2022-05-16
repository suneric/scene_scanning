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
  for (int i = 0; i < VoxelCount(); ++i)
  {
    WSPoint point = VoxelCentroid(i);
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

WSPointCloudNormalPtr PCLOctree::VoxelAverageNormals_All(
  const Eigen::Vector3f& refViewpoint,
  WSPointCloudPtr& voxelCentroid) const
{
  if (nullptr == m_centroidCloud)
    return nullptr;

  std::cout << "evaluate surface normal with all points in the voxel \n";

  std::vector<Eigen::Vector3f> centroidVec;
  std::vector<Eigen::Vector3f> normalVec;

  PCLFilter filter;
  for (int i = 0; i < VoxelCount(); ++i)
  {
    WSPoint point = VoxelCentroid(i);
    std::vector<int> voxelIndices;
    if (!m_os->voxelSearch(point,voxelIndices))
      continue;

    // evaluate average normal of the voxel
    WSPointCloudPtr vCloud = filter.ExtractPoints(m_cloud, voxelIndices);
    WSPointCloudNormalPtr vNormal(new WSPointCloudNormal);
    pcl::NormalEstimation<WSPoint, WSNormal> ne;
    pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(vCloud);
    ne.setViewPoint(refViewpoint.x(),refViewpoint.y(),refViewpoint.z());
    //ne.setKSearch(10);
    double rad = VoxelSideLength();
    ne.setRadiusSearch(rad);
    ne.compute(*vNormal);

    // find average normal for the voxel
    Eigen::Vector3f vec(0,0,0);
    int numCount = vNormal->points.size();
    for (size_t j = 0; j < numCount; ++j)
    {
      WSNormal normal = vNormal->points[j];
      vec += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
    }
    Eigen::Vector3f nm = (vec/numCount).normalized();

    // check visibility
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

bool PCLOctree::IsOutsideVoxel(const WSPoint& point, const Eigen::Vector3f& refNormal)
{
  int outsideSize = 0;
  double halfVLen = 0.5*m_os->getVoxelSquaredSideLen();
  Eigen::Vector3f c(point.x,point.y,point.z);
  // x direction
  std::vector<Eigen::Vector3f> checkPoints;
  checkPoints.push_back(Eigen::Vector3f(c.x()-2*halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x()+2*halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()-2*halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()+2*halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()-2*halfVLen));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()+2*halfVLen));
  for (int i = 0; i < checkPoints.size(); ++i)
  {
    if (refNormal.dot(checkPoints[i]) >= 0.0)
    {
      int nInterSectVoxel = IntersectedOccupiedVoxels(c,checkPoints[i]);
      if (nInterSectVoxel == 1)
        outsideSize++;
    }
  }

  return outsideSize > 0;
}

int PCLOctree::BoxSearch(const Eigen::Vector3f& minPt,
              const Eigen::Vector3f& maxPt,
              std::vector<int>& indices) const
{
  return m_os->boxSearch(minPt, maxPt, indices);
}

namespace
{
  WSPointCloudPtr Polygon_X(const Eigen::Vector3f& center, double halfVLen, double flag)
  {
    WSPointCloudPtr polygon(new WSPointCloud);
    double xoffset = flag*halfVLen;
    polygon->points.resize(5);
    polygon->points[0].x = center.x()+xoffset;
    polygon->points[0].y = center.y()-halfVLen;
    polygon->points[0].z = center.z()-halfVLen;

    polygon->points[1].x = center.x()+xoffset;
    polygon->points[1].y = center.y()-halfVLen;
    polygon->points[1].z = center.z()+halfVLen;

    polygon->points[2].x = center.x()+xoffset;
    polygon->points[2].y = center.y()+halfVLen;
    polygon->points[2].z = center.z()+halfVLen;

    polygon->points[3].x = center.x()+xoffset;
    polygon->points[3].y = center.y()+halfVLen;
    polygon->points[3].z = center.z()-halfVLen;

    polygon->points[4].x = center.x()+xoffset;
    polygon->points[4].y = center.y()-halfVLen;
    polygon->points[4].z = center.z()-halfVLen;
    return polygon;
  }

  WSPointCloudPtr Polygon_Y(const Eigen::Vector3f& center, double halfVLen, double flag)
  {
    WSPointCloudPtr polygon(new WSPointCloud);
    double yoffset = flag*halfVLen;
    polygon->points.resize(5);
    polygon->points[0].x = center.x()-halfVLen;
    polygon->points[0].y = center.y()+yoffset;
    polygon->points[0].z = center.z()-halfVLen;

    polygon->points[1].x = center.x()-halfVLen;
    polygon->points[1].y = center.y()+yoffset;
    polygon->points[1].z = center.z()+halfVLen;

    polygon->points[2].x = center.x()+halfVLen;
    polygon->points[2].y = center.y()+yoffset;
    polygon->points[2].z = center.z()+halfVLen;

    polygon->points[3].x = center.x()+halfVLen;
    polygon->points[3].y = center.y()+yoffset;
    polygon->points[3].z = center.z()-halfVLen;

    polygon->points[4].x = center.x()-halfVLen;
    polygon->points[4].y = center.y()+yoffset;
    polygon->points[4].z = center.z()-halfVLen;
    return polygon;
  }

  WSPointCloudPtr Polygon_Z(const Eigen::Vector3f& center, double halfVLen, double flag)
  {
    WSPointCloudPtr polygon(new WSPointCloud);
    double zoffset = flag*halfVLen;
    polygon->points.resize(5);
    polygon->points[0].x = center.x()-halfVLen;
    polygon->points[0].y = center.y()-halfVLen;
    polygon->points[0].z = center.z()+zoffset;

    polygon->points[1].x = center.x()-halfVLen;
    polygon->points[1].y = center.y()+halfVLen;
    polygon->points[1].z = center.z()+zoffset;

    polygon->points[2].x = center.x()+halfVLen;
    polygon->points[2].y = center.y()+halfVLen;
    polygon->points[2].z = center.z()+zoffset;

    polygon->points[3].x = center.x()+halfVLen;
    polygon->points[3].y = center.y()-halfVLen;
    polygon->points[3].z = center.z()+zoffset;

    polygon->points[4].x = center.x()-halfVLen;
    polygon->points[4].y = center.y()-halfVLen;
    polygon->points[4].z = center.z()+zoffset;
    return polygon;
  }

};

void PCLOctree::FindOutsidePolygons(std::vector<WSPointCloudPtr>& outsidePolygons) const
{
  int nInterSectVoxel = 0;
  double halfVLen = 0.5*m_os->getVoxelSquaredSideLen();
  for (int i = 0; i < m_centroidCloud->points.size(); ++i)
  {
    WSPoint c = m_centroidCloud->points[i];
    Eigen::Vector3f center(c.x,c.y,c.z);

    // x direction
    Eigen::Vector3f xE1(c.x-2*halfVLen,c.y,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,xE1);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_X(center,halfVLen,-1.0);
      outsidePolygons.push_back(polygon);
    }

    Eigen::Vector3f xE2(c.x+2*halfVLen,c.y,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,xE2);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_X(center,halfVLen,1.0);
      outsidePolygons.push_back(polygon);
    }

    // y direction
    Eigen::Vector3f yE1(c.x,c.y-2*halfVLen,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,yE1);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_Y(center,halfVLen,-1.0);
      outsidePolygons.push_back(polygon);
    }

    Eigen::Vector3f yE2(c.x,c.y+2*halfVLen,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,yE2);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_Y(center,halfVLen,1.0);
      outsidePolygons.push_back(polygon);
    }

    // z direction
    Eigen::Vector3f zE1(c.x,c.y,c.z-2*halfVLen);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,zE1);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_Z(center,halfVLen,-1.0);
      outsidePolygons.push_back(polygon);
    }

    Eigen::Vector3f zE2(c.x,c.y,c.z+2*halfVLen);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,zE2);
    if (nInterSectVoxel == 1)
    {
      WSPointCloudPtr polygon = Polygon_Z(center,halfVLen,1.0);
      outsidePolygons.push_back(polygon);
    }
  }
}

int PCLOctree::IntersectedOccupiedVoxels(const Eigen::Vector3f& origin, const Eigen::Vector3f& end) const
{
  pcl::octree::OctreePointCloud<WSPoint>::AlignedPointTVector tmpCentroids;
  int nInterSectVoxel = m_os->getApproxIntersectedVoxelCentersBySegment(origin,end,tmpCentroids);
  int count = 0;
  for (size_t j = 0; j < tmpCentroids.size(); ++j)
  {
    bool bOccupied = m_os->isVoxelOccupiedAtPoint(tmpCentroids[j]);
    if (bOccupied)
      count++;
  }
  //std::cout << "intersected:" << tmpCentroids.size() << "occupied" << count << std::endl;
  return count;
}

void PCLOctree::VoxelOutsideCenters(std::vector<VoxelNormals>& outsideNormals) const
{
  int nInterSectVoxel = 0;
  double halfVLen = 0.5*m_os->getVoxelSquaredSideLen();
  for (int i = 0; i < m_centroidCloud->points.size(); ++i)
  {
    WSPoint c = m_centroidCloud->points[i];
    Eigen::Vector3f center(c.x,c.y,c.z);
    std::vector<Eigen::Vector3f> outsideCenters;

    // x direction
    Eigen::Vector3f xE1(c.x-2*halfVLen,c.y,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,xE1);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x-halfVLen,c.y,c.z));

    Eigen::Vector3f xE2(c.x+2*halfVLen,c.y,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,xE2);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x+halfVLen,c.y,c.z));

    // y direction
    Eigen::Vector3f yE1(c.x,c.y-2*halfVLen,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,yE1);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x,c.y-halfVLen,c.z));

    Eigen::Vector3f yE2(c.x,c.y+2*halfVLen,c.z);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,yE2);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x,c.y+halfVLen,c.z));

    // z direction
    Eigen::Vector3f zE1(c.x,c.y,c.z-2*halfVLen);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,zE1);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x,c.y,c.z-halfVLen));

    Eigen::Vector3f zE2(c.x,c.y,c.z+2*halfVLen);
    nInterSectVoxel = IntersectedOccupiedVoxels(center,zE2);
    if (nInterSectVoxel == 1)
      outsideCenters.push_back(Eigen::Vector3f(c.x,c.y,c.z+halfVLen));

    outsideNormals.push_back(std::make_pair(center, outsideCenters));
  }
}
