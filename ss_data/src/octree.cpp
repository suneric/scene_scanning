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

WSPointCloudNormalPtr PCLOctree::VoxelAverageNormals(
  const Eigen::Vector3f& refNormal,
  const std::vector<double>& bbox,
  WSPointCloudPtr& voxelCentroid) const
{
  if (nullptr == m_centroidCloud)
    return nullptr;

  std::vector<int> voxelIndices;
  int nVoxel = m_voxelMap.VoxelIndices(voxelIndices);
  std::vector<WSPoint> subsetCentroids;
  for (int i = 0; i < nVoxel; ++i)
  {
    WSPoint point = m_voxelMap.GetVoxelCentroid(i);
    if (InBoundingBox(point,bbox))
      subsetCentroids.push_back(point);
  }

  int nSubset = subsetCentroids.size();
  WSPointCloudPtr centroids(new WSPointCloud);
  centroids->points.resize(nSubset);
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  normals->points.resize(nSubset);
  for (int i = 0; i < nSubset; ++i)
  {
    WSPoint point = subsetCentroids[i];
    centroids->points[i] = point;
    WSNormal normal;
    EvaluateVoxelNormal(m_cloud, point, normal, refNormal);
    normals->points[i].normal_x = normal.normal_x;
    normals->points[i].normal_y = normal.normal_y;
    normals->points[i].normal_z = normal.normal_z;
  }
  voxelCentroid = centroids;
  return normals;
}

// evaluete average voxel normal, return if fliped
bool PCLOctree::EvaluateVoxelNormal(const WSPointCloudPtr cloud, const WSPoint& point, WSNormal& normal, const Eigen::Vector3f& refNormal) const
{
  bool bFlip = false;
  std::vector<int> voxelIndices;
  if (m_os->voxelSearch(point,voxelIndices))
  {
    // find a all point grouped in the voxel and evaluate point normals
    WSPointCloudNormalPtr normals(new WSPointCloudNormal);

    PCLFilter filter;
    WSPointCloudPtr groupPt = filter.ExtractPoints(cloud,voxelIndices);
    pcl::NormalEstimation<WSPoint, WSNormal> ne;
    pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(groupPt);
    //ne.setKSearch(10);
    ne.setRadiusSearch(0.1); // use all neighbors in 10 cm
    ne.compute(*normals);

    // find average normal for the voxel
    Eigen::Vector3f vec(0,0,0);
    size_t num = normals->points.size();
    for (size_t i = 0; i < num; ++i)
    {
      WSNormal normal = normals->points[i];
      vec += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
    }
    //std::cout << "normal vector " << num << " " << vec.x() << " " << vec.y() << " " << vec.z() << std::endl;
    Eigen::Vector3f nm = (vec/num).normalized();
    if (nm.dot(refNormal) < 0.0) // normal angle > M_PI/2
    {
      nm = -nm;
      bFlip = true;
    }
    normal.normal_x = nm.x();
    normal.normal_y = nm.y();
    normal.normal_z = nm.z();
  }
  else
  {
    std::cout << "no search result" << std::endl;
  }
  return bFlip;
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
              std::vector<int> indices) const
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

// void CreateViewpointsWithAverageNormal(const WSPointCloudPtr cloud, OctreeSearch os, double distance, std::vector<Eigen::Affine3f>& cameras)
// {
//   // find all voxel centroids and average normal of the voxel
//   PCLOctree octree;
//   WSPointCloudNormalPtr vNormal(new WSPointCloudNormal);
//   WSPointCloudPtr vCloud = octree.EvaluateNormals(cloud, os, vNormal);
//
//   // find all camera poses
//   std::vector<Eigen::Affine3f> cameraPoseVec;
//   for (size_t i = 0; i < vCloud->points.size(); ++i)
//   {
//     Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
//     Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
//     Eigen::Vector3f camera;
//     if (CameraPosition(point, nm, os.getResolution(), distance, camera))
//       cameras.push_back(octree.CameraPose(camera,-nm));
//   }
// }
//
// void CubeFaceCenter(const Eigen::Vector3f& center, double sideLen, std::vector<Eigen::Vector3f>& faceCenters)
// {
//   Eigen::Vector3f v1(center.x()-0.5*sideLen,center.y(),center.z());
//   Eigen::Vector3f v2(center.x()+0.5*sideLen,center.y(),center.z());
//   Eigen::Vector3f v3(center.x(),center.y()-0.5*sideLen,center.z());
//   Eigen::Vector3f v4(center.x(),center.y()+0.5*sideLen,center.z());
//   Eigen::Vector3f v5(center.x(),center.y(),center.z()-0.5*sideLen);
//   Eigen::Vector3f v6(center.x(),center.y(),center.z()+0.5*sideLen);
//   faceCenters.push_back(v1);
//   faceCenters.push_back(v2);
//   faceCenters.push_back(v3);
//   faceCenters.push_back(v4);
//   faceCenters.push_back(v5);
//   faceCenters.push_back(v6);
// }
//
// void CreateViewpointsWithVoxelCube(const WSPointCloudPtr cloud, OctreeSearch os, double distance, std::vector<Eigen::Affine3f>& cameras)
// {
//   PCLOctree octree;
//   double voxelLen = sqrt(os.getVoxelSquaredSideLen());
//   for (size_t i = 0; i < cloud->points.size(); ++i)
//   {
//     Eigen::Vector3f center(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//     std::vector<Eigen::Vector3f> faceCenters;
//     CubeFaceCenter(center,voxelLen,faceCenters);
//     for (size_t j = 0; j < faceCenters.size(); j++)
//     {
//       Eigen::Vector3f nm = (faceCenters[j]-center).normalized();
//       std::vector<int> intersectIndices;
//       int nInterSectVoxel = os.getIntersectedVoxelIndices(center,nm,intersectIndices);
//       if (nInterSectVoxel <= 1)
//       {
//         Eigen::Vector3f camera;
//         if (CameraPosition(center, nm, os.getResolution(), distance, camera))
//           cameras.push_back(octree.CameraPose(camera,-nm));
//       }
//     }
//   }
// }
