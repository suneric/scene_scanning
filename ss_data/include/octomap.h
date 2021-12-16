#ifndef _SSV3D_PCL_OCTOMAP_H_
#define _SSV3D_PCL_OCTOMAP_H_

#include "viewer.h"

namespace ssv3d
{
  class Voxel3d
  {
  public:
    Voxel3d(int id, double res, const Eigen::Vector3f& vMin, const Eigen::Vector3f& vMax)
    {
      m_status = 0;
      m_id = id;
      m_center = WSPoint();
      m_center.x = 0.5*(vMin.x()+vMax.x());
      m_center.y = 0.5*(vMin.y()+vMax.y());
      m_center.z = 0.5*(vMin.z()+vMax.z());
      m_length = res;
      m_centroid = m_center;
      m_normal = WSNormal(0,0,0);
      m_ptIndices.clear();
    }

    Voxel3d(const Voxel3d& source)
    {
      m_status = source.m_status;
      m_id = source.m_id;
      m_center = source.m_center;
      m_length = source.m_length;
      m_centroid = source.m_centroid;
      m_normal = source.m_normal;
      m_ptIndices.assign(source.m_ptIndices.begin(), source.m_ptIndices.end());
    }

    Voxel3d& operator=(const Voxel3d& source)
    {
      if (this != &source)
      {
        m_status = source.m_status;
        m_id = source.m_id;
        m_center = source.m_center;
        m_length = source.m_length;
        m_centroid = source.m_centroid;
        m_normal = source.m_normal;
        m_ptIndices.assign(source.m_ptIndices.begin(), source.m_ptIndices.end());
      }
      return *this;
    }

    void SetStatus(int status){m_status = status;}
    void AddCentroid(const WSPoint& centroid){m_centroid = centroid;}
    void AddNormal(const WSNormal& normal){m_normal = normal;}
    void AddPointIndex(int index){m_ptIndices.push_back(index);}
    void PointIndices(std::vector<int>& indices){indices.assign(m_ptIndices.begin(), m_ptIndices.end());}
    bool InVoxel(const WSPoint& pt) const
    {
      double minx = m_center.x-0.5*m_length, miny = m_center.y-0.5*m_length, minz = m_center.z-0.5*m_length;
      double maxx = m_center.x+0.5*m_length, maxy = m_center.y+0.5*m_length, maxz = m_center.z+0.5*m_length;
      if (pt.x > maxx || pt.x < minx)
        return false;
      if (pt.y > maxy || pt.y < miny)
        return false;
      if (pt.z > maxz || pt.z < minz)
        return false;
      return true;
    }

    ~Voxel3d(){}

    int Id() const {return m_id;}
    int Status() const {return m_status;}
    WSPoint Center() const {return m_center;}
    double Length() const {return m_length;}
    WSPoint Centroid() const {return m_centroid;}
    WSNormal Normal() const {return m_normal;}
    bool Empty() const {return m_ptIndices.empty();}

  private:
    int m_status; // 0: unknown, 1: free, 2: occuplied, 3 :occluded
    int m_id;
    double m_length;
    WSPoint m_center;
    WSPoint m_centroid;
    WSNormal m_normal;
    std::vector<int> m_ptIndices;
  };

  class OctoMap
  {
  public:
    OctoMap(double resolution);
    ~OctoMap();

    void CreateMap(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
    void AddView(const Eigen::Matrix4f& viewPoint, const WSPointCloudPtr& cloud);

    void OccupiedVoxels(std::vector<Voxel3d>& voxels);
    void FreeVoxels(std::vector<Voxel3d>& voxels);
    void OccludedVoxels(std::vector<Voxel3d>& voxels);

    bool VoxelIndex(const WSPoint& pt, int& voxelIndex);
    void ViewVoxels(const Eigen::Matrix4f& viewPoint, std::vector<int>& voxelIndices);
    bool FrustumCulling(const Eigen::Matrix4f& view, WSPointCloudPtr viewCloud);
    Eigen::Matrix4f CameraPoseTransform(const Eigen::Matrix4f& mat);

    void AddPointToVoxel(const std::vector<int>& voxelIndices, const WSPointCloudPtr& cloud, std::vector<int>& occupiedIndices);
    void AddCentroidToVoxel(const std::vector<int>& voxelIndices);
    void AddNormalToVoxel(const std::vector<int>& voxelIndices, const Eigen::Matrix4f& viewPoint);
    void AddStatusToVoxel(const Eigen::Matrix4f& viewPoint, const std::vector<int>& voxelIndices);
    void RayTrace(const Eigen::Vector3f& origin, const Eigen::Vector3f& target, const std::vector<int>& voxelIndices, std::vector<int>& intersectedVoxels);
    bool CheckIntersection(const Eigen::Vector3f& check, std::vector<int>& voxelIndices, int& intersectedVoxel);

    WSPointCloudPtr PointCloud() const;

  private:
    double m_resolution;
    std::vector<WSPoint> m_points;
    std::vector<Voxel3d> m_voxels;
    WSPointCloudPtr m_centerCloud;
  };
};


#endif //_SSV3D_PCL_OCTOMAP_H_
