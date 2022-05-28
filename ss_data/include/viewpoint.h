#ifndef _SSV3D_PCL_VIEWPOINT_H_
#define _SSV3D_PCL_VIEWPOINT_H_

#include "octree.h"

namespace ssv3d
{
  struct Cartesion
  {
    Cartesion(){pos_x=pos_y=pos_z=ori_x=ori_y=ori_z=ori_w=0;}
    Cartesion(const Cartesion& p)
    {
      pos_x = p.pos_x;
      pos_y = p.pos_y;
      pos_z = p.pos_z;
      ori_x = p.ori_x;
      ori_y = p.ori_y;
      ori_z = p.ori_z;
      ori_w = p.ori_w;
    }
    Cartesion& operator=(const Cartesion& p)
    {
      pos_x = p.pos_x;
      pos_y = p.pos_y;
      pos_z = p.pos_z;
      ori_x = p.ori_x;
      ori_y = p.ori_y;
      ori_z = p.ori_z;
      ori_w = p.ori_w;
      return *this;
    }
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;
  };

  class PCLViewPoint{
  public:
    void GenerateCameraPositions(
      const PCLOctree& tree,
      const std::vector<int>& vxIndices,
      double distance,
      double height_min,
      double height_max,
      const Eigen::Vector3f& refViewpoint,
      bool bSampling,
      std::vector<Eigen::Affine3f>& cameras
    );

    WSPointCloudPtr CameraViewVoxels(
      const PCLOctree& tree,
      const Eigen::Affine3f& camera,
      std::vector<int>& voxelInices
    );
    void SaveToFile(
      const std::string& output,
      std::vector<Eigen::Affine3f>& cameras,
      std::map<int, std::vector<int> >& voxelMap
    );
    void LoadFromFile(
      const std::string& input,
      std::vector<Eigen::Affine3f>& cameras
    );
    bool FilterViewPoint(const PCLOctree& tree, const Eigen::Affine3f& camera, double heightMin, double heightMax);

    bool QuadrotorBBox(const Eigen::Affine3f& camera, Eigen::Vector3f& minPt, Eigen::Vector3f& maxPt);

  private:
    Eigen::Affine3f ViewPoint2CameraPose(const Cartesion& vp);
    Cartesion CameraPose2ViewPoint(const Eigen::Affine3f& camera);
    // convert traditional x right, y down and z forward matrix to xforward,y up, z right.
    Eigen::Matrix4f CameraPoseTransform(const Eigen::Matrix4f& mat);
    bool IsVisibleVoxel(const PCLOctree& tree, const Eigen::Vector3f& camera, const Eigen::Vector3f& centroid);
    Eigen::Affine3f CameraPosition(const Eigen::Vector3f& target, const Eigen::Vector3f& normal, double distance, double height_min, double height_max);
    Eigen::Affine3f CameraMatrix(const Eigen::Vector3f& center, const Eigen::Vector3f& normal);
    bool FrustumCulling(const WSPointCloudPtr cloud,const Eigen::Matrix4f& camera,float hfov, float vfov,float ndist, float fdist,WSPointCloudPtr viewPoints);
    Eigen::Matrix4f Quadrotor2Camera(double camera_angle);
  };
};

#endif //_SSV3D_PCL_VIEWPOINT_H_
