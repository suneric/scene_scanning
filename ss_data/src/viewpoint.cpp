#include "viewpoint.h"
#include "filter.h"
#include <math.h>
#include <fstream>
#include <sstream>

#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>

using namespace ssv3d;

void PCLViewPoint::GenerateCameraPositions(
  const PCLOctree& tree,
  double distance,
  const Eigen::Vector3f& refNormal,
  const std::vector<double>& bbox,
  std::vector<Eigen::Affine3f>& cameras,
  double height_min,
  double height_max)
{
  WSPointCloudPtr vCloud;
  WSPointCloudNormalPtr vNormal = tree.VoxelAverageNormals(refNormal,bbox,vCloud);
  for (size_t i = 0; i < vCloud->points.size(); ++i)
  {
    Eigen::Vector3f point(vCloud->points[i].x, vCloud->points[i].y, vCloud->points[i].z);
    Eigen::Vector3f nm(vNormal->points[i].normal_x, vNormal->points[i].normal_y, vNormal->points[i].normal_z);
    Eigen::Affine3f camera = CameraPosition(point,nm,distance,height_min,height_max);
    if (!FilterViewPoint(tree,camera))
      cameras.push_back(camera);
  }
}

// the camere position should be placed at safe distance to the aircraft which is longer than the defined distance
// consider the robot workspace is limited, also need to consider the height (z) of the robot can reach
Eigen::Affine3f PCLViewPoint::CameraPosition(const Eigen::Vector3f& target,const Eigen::Vector3f& normal,double distance, double height_min, double height_max)
{
  Eigen::Vector3f nm = normal.normalized();
  Eigen::Vector3f pt = target + distance*nm;
  if (pt.z() < 0.5)
    pt = target + 0.5*distance*nm;

  return CameraMatrix(pt,-nm);
}

// give a position of camera and the direction of camera facing
// calculate the orientation of camera
// given the condition that rolling of camera is 0 as uav is hovering
Eigen::Affine3f PCLViewPoint::CameraMatrix(const Eigen::Vector3f& center, const Eigen::Vector3f& normal)
{
  Eigen::Vector3f nm = normal.normalized();
  double beta = acos(nm.z()); // pitch angle
  double alpha = 0.0;
  if (nm.x() != 0)
    alpha = atan2(nm.y(),nm.x()); // yaw angle

  Eigen::Matrix4f mat;
  mat(0,0) = sin(alpha);
  mat(0,1) = cos(alpha)*cos(beta);
  mat(0,2) = cos(alpha)*sin(beta);
  mat(0,3) = center.x();
  mat(1,0) = -cos(alpha);
  mat(1,1) = sin(alpha)*cos(beta);
  mat(1,2) = sin(alpha)*sin(beta);
  mat(1,3) = center.y();
  mat(2,0) = 0;
  mat(2,1) = -sin(beta);
  mat(2,2) = cos(beta);
  mat(2,3) = center.z();
  mat(3,0) = 0;
  mat(3,1) = 0;
  mat(3,2) = 0;
  mat(3,3) = 1;
  Eigen::Affine3f res;
  res.matrix() = mat;
  return res;
}

bool PCLViewPoint::FilterViewPoint(const PCLOctree& tree, const Eigen::Affine3f& camera)
{
  // no point in view frustum
  std::vector<int> voxelIndices;
  CameraViewVoxels(tree,camera,voxelIndices);
  if (voxelIndices.empty())
    return true;

  return false;
}

WSPointCloudPtr PCLViewPoint::CameraViewVoxels(const PCLOctree& tree, const Eigen::Affine3f& camera, std::vector<int>& voxelIndices)
{
  Eigen::Matrix4f camMatrix = camera.matrix();
  Eigen::Vector3f cameraPt(camMatrix(0,3),camMatrix(1,3),camMatrix(2,3));
  // get voxels in camera frustum culling
  WSPointCloudPtr viewCloud(new WSPointCloud);
  if(FrustumCulling(tree.VoxelCentroidCloud(),camera.matrix(),80,80,0.1,5.0,viewCloud))
  {
    std::vector<WSPoint> visibleVoxels;
    for (int i = 0; i < viewCloud->points.size(); ++i)
    {
      WSPoint c = viewCloud->points[i];
      if (IsVisibleVoxel(tree,cameraPt,Eigen::Vector3f(c.x,c.y,c.z)))
        visibleVoxels.push_back(c);
    }

    // voxel index
    std::vector<WSPoint> visibleCentroids;
    for (size_t i = 0; i < visibleVoxels.size(); ++i)
    {
      int voxelIndex = tree.VoxelIndex(visibleVoxels[i]);
      if (std::find(voxelIndices.begin(),voxelIndices.end(),voxelIndex) == voxelIndices.end())
      {
        voxelIndices.push_back(voxelIndex);
        visibleCentroids.push_back(visibleVoxels[i]);
      }
    }

    WSPointCloudPtr visibleVoxelCloud(new WSPointCloud);
    visibleVoxelCloud->points.resize(visibleCentroids.size());
    visibleVoxelCloud->points.assign(visibleCentroids.begin(), visibleCentroids.end());
    return visibleVoxelCloud;
  }
  else
  {
    std::cout << "no voxels in camera view" << std::endl;
    return nullptr;
  }
}

// check if voxel is blocked by other voxel in the view with checking all of its side
bool PCLViewPoint::IsVisibleVoxel(const PCLOctree& tree, const Eigen::Vector3f& camera, const Eigen::Vector3f& c)
{
  int visibleSide = 0;
  double halfVLen = 0.5*tree.VoxelSideLength();
  std::vector<Eigen::Vector3f> checkPoints;
  checkPoints.push_back(Eigen::Vector3f(c.x()-halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x()+halfVLen,c.y(),c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()-halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y()+halfVLen,c.z()));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()-halfVLen));
  checkPoints.push_back(Eigen::Vector3f(c.x(),c.y(),c.z()+halfVLen));
  for (int i = 0; i < checkPoints.size(); ++i)
  {
    std::vector<WSPoint> intesects;
    int nRes = tree.IntersectedOccupiedVoxels(camera, c);
    //std::cout << "intersects " << nRes << std::endl;
    if (nRes <= 1) // consider the two voxel share the same edge
      visibleSide++;
  }
  //std::cout << "visible side " << visibleSide << std::endl;
  return visibleSide > 0;
}

bool PCLViewPoint::FrustumCulling(const WSPointCloudPtr cloud,
                               const Eigen::Matrix4f& cpose,
                               float hfov, float vfov,
                               float ndist, float fdist,
                               WSPointCloudPtr viewCloud)
{
  if (cloud == nullptr)
    return false;

  Eigen::Matrix4f poseNew = CameraPoseTransform(cpose);
  // frustum culling
  pcl::FrustumCulling<WSPoint> fc;
  fc.setInputCloud(cloud);
  fc.setCameraPose(poseNew);
  fc.setHorizontalFOV(hfov); // degree
  fc.setVerticalFOV(vfov); // degree
  fc.setNearPlaneDistance(ndist);
  fc.setFarPlaneDistance(fdist);
  fc.filter(*viewCloud);
  return true;
}

// This assume a coordinate system where X is forward, Y is up and Z is right.
// to convert from the traditional camera coordinate syste (X is right, Y is down, Z forward)
// [0,0,1,0
//  0,-1,0,0
//  1,0,0,0
//  0,0,0,1]
Eigen::Matrix4f PCLViewPoint::CameraPoseTransform(const Eigen::Matrix4f& mat)
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

void PCLViewPoint::SaveToFile(const std::string& output,
                             std::vector<Eigen::Affine3f>& cameras,
                             std::map<int, std::vector<int> >& voxelMap)
{
  std::ofstream tFile(output);
  int vpIndex = 0;
  for (size_t i = 0; i < cameras.size(); ++i)
  {
    if (voxelMap[i].size() == 0){
      continue;
    }

    Cartesion vp = CameraPose2ViewPoint(cameras[i]);
    // each line: viewpoint_idx px py pz ox oy oz ow voxel_indices ... \n
    tFile << vpIndex << " " << vp.pos_x << " "
                      << vp.pos_y << " "
                      << vp.pos_z << " "
                      << vp.ori_x << " "
                      << vp.ori_y << " "
                      << vp.ori_z << " "
                      << vp.ori_w << " ";
    for (size_t j = 0; j < voxelMap[i].size(); ++j)
    {
      int vIndex = voxelMap[i][j];
      tFile << vIndex;
      if (j == voxelMap[i].size()-1)
        tFile << "\n";
      else
        tFile << " ";
    }
    vpIndex++;
  }
  tFile.close();
}

void PCLViewPoint::LoadFromFile(const std::string& input, std::vector<Eigen::Affine3f>& cameras)
{
  std::ifstream tFile(input);
  std::string line;
  while (std::getline(tFile, line))
  {
    std::stringstream linestream(line);
    std::string index,px,py,pz,ox,oy,oz,ow,angle;
    linestream >> index >> px >> py >> pz >> ox >> oy >> oz >> ow >> angle;
    Cartesion vp;
    vp.pos_x = std::stod(px);
    vp.pos_y = std::stod(py);
    vp.pos_z = std::stod(pz);
    vp.ori_x = std::stod(ox);
    vp.ori_y = std::stod(oy);
    vp.ori_z = std::stod(oz);
    vp.ori_w = std::stod(ow);
    Eigen::Affine3f camera = ViewPoint2CameraPose(vp);
    cameras.push_back(camera);
  }
  tFile.close();
}

Cartesion PCLViewPoint::CameraPose2ViewPoint(const Eigen::Affine3f& camera)
{
  Cartesion vp;
  Eigen::Matrix3f affine = camera.matrix().topLeftCorner(3,3);
  Eigen::Quaternionf q(affine);
  vp.pos_x = camera(0,3);
  vp.pos_y = camera(1,3);
  vp.pos_z = camera(2,3);
  vp.ori_w = q.w();
  vp.ori_x = q.x();
  vp.ori_y = q.y();
  vp.ori_z = q.z();
  return vp;
}

Eigen::Affine3f PCLViewPoint::ViewPoint2CameraPose(const Cartesion& vp)
{
  Eigen::Vector3f t;
  Eigen::Quaternionf q;
  t.x() = vp.pos_x;
  t.y() = vp.pos_y;
  t.z() = vp.pos_z;
  q.x() = vp.ori_x;
  q.y() = vp.ori_y;
  q.z() = vp.ori_z;
  q.w() = vp.ori_w;
  Eigen::Matrix3f rot = q.normalized().toRotationMatrix();

  Eigen::Matrix4f cam_pose;
  cam_pose.setIdentity();
  cam_pose.block<3,3>(0,0)=rot;
  cam_pose.block<3,1>(0,3)=t;

  Eigen::Affine3f res;
  res.matrix() = cam_pose;
  return res;
}
