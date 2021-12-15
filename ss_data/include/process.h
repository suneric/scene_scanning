#ifndef _SSDATA_PCL_PROCESS_H_
#define _SSDATA_PCL_PROCESS_H_

#include "octomap.h"
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
    bool RegisterPointCloud(OctoMap* map);

  private:
    std::vector<std::vector<double> > m_viewpointList;
    std::vector<WSPointCloudPtr> m_cloudList;
  };
};

#endif //!_SSDATA_PCL_VIEWER_H_
