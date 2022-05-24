#ifndef _SSV3D_PCL_DISPLAY_H_
#define _SSV3D_PCL_DISPLAY_H_

#include "viewer.h"
#include "viewpoint.h"
#include <string.h>

namespace ssv3d {

  class PCDisplay
  {
    public:
      WSPointCloudPtr LoadPointCloud(const std::string& file);
      bool SavePointCloud(const WSPointCloudPtr cloud, const std::string& dir);
      void LoadViewpoints(const std::string& file, std::vector<Eigen::Affine3f>& cameras);
      Eigen::Affine3f ViewPoint2CameraPose(const Cartesion& vp);
  };

};

#endif //_SSV3D_PCL_DISPLAY_H_
