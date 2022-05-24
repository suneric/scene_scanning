#include "display.h"
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>


using namespace ssv3d;

Eigen::Affine3f PCDisplay::ViewPoint2CameraPose(const Cartesion& vp)
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

WSPointCloudPtr PCDisplay::LoadPointCloud(const std::string& file)
{
  WSPointCloudPtr cloud(new WSPointCloud());
  int res = pcl::io::loadPCDFile(file, *cloud);
  if (res >= 0)
    return cloud;
  else
    return nullptr;
}

bool PCDisplay::SavePointCloud(const WSPointCloudPtr cloud, const std::string& dir)
{
  if (cloud == nullptr)
    return false;

  std::string filePath = dir+"point_cloud.pcd";
  int res = pcl::io::savePCDFileASCII(filePath, *cloud);
  if (res >= 0)
    std::cout << "save as " << filePath << std::endl;
  return res >= 0;
}

void PCDisplay::LoadViewpoints(const std::string& file, std::vector<Eigen::Affine3f>& cameras)
{
  std::ifstream tFile(file);
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
