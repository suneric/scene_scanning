#ifndef _SSV3D_PCL_FILTER_H_
#define _SSV3D_PCL_FILTER_H_

#include "viewer.h"

namespace ssv3d {

  class PCLFilter {
    public:
      WSPointCloudPtr FilterPCLPointInBBox(const WSPointCloudPtr cloud, const std::vector<double>& bbox, bool bFilter = false);
      WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize);
      WSPointCloudPtr FilterPassThrough(const WSPointCloudPtr cloud, const std::string& field, double limit_min, double limit_max);
      WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh);

      WSPointCloudPtr SamplingSurfaceNormal(const WSPointCloudPtr cloud, unsigned int sample, float ratio, WSPointCloudNormalPtr& normals);
      // create sub sampling point cloud with number of sample
      bool RandomSampling(const WSPointCloudPtr cloud, unsigned int sample, std::vector<int>& indices);
      // create a uniform sampling point cloud with a search radius
      WSPointCloudPtr UniformSampling(const WSPointCloudPtr cloud, double radius);

      WSPointCloudPtr ExtractPoints(const WSPointCloudPtr cloud, const std::vector<int>& indices);
      WSPointCloudNormalPtr ExtractNormals(const WSPointCloudNormalPtr normal, const std::vector<int>& indices);

      // extract point on the slicing plane
      WSPointCloudPtr SlicePoints(const WSPointCloudPtr cloud, const Eigen::Vector3f& rootPt, const Eigen::Vector3f& normal);
      WSPointCloudPtr SortPointsInZ(const WSPointCloudPtr cloud, double resolution, double min, double max);
      WSPointCloudPtr NeighborPoints(const WSPoint& pt, const WSPointCloudPtr cloud, double dRadius);
      WSNormal PointNormal(const WSPoint& pt, const WSPointCloudPtr cloud, double dRadius, const Eigen::Vector3f& refNormal);
  };

};

#endif //_SSV3D_PCL_FILTER_H_
