#ifndef _SSDATA_PCL_PROCESS_H_
#define _SSDATA_PCL_PROCESS_H_

#include "viewer.h"
#include "std_msgs/Float64MultiArray.h"

namespace ssv3d
{
  class PCLProcess
  {
  public:
    PCLProcess();
    ~PCLProcess();

    void AddViewPoint(const std::vector<double>& vp);
    void AddPointCloud(const WSPointCloudPtr& cloud);

    WSPointCloudPtr PointCloud() const;
    bool Registration();
    void PairAlign (const WSPointCloudPtr cloud_src,
      WSPointCloudPtr cloud_tgt,
      WSPointCloudPtr output,
      Eigen::Matrix4f &final_transform,
      bool downsample = false);

  private:
    WSPointCloudPtr m_cloud;
    Eigen::Matrix4f m_globalTransform;
    std::vector<std::vector<double> > m_viewpointList;
    std::vector<WSPointCloudPtr> m_cloudList;
  };
};

#endif //!_SSDATA_PCL_VIEWER_H_
