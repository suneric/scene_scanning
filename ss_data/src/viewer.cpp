#include <math.h>
#include <vector>
#include "viewer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/common/common.h>
#include <boost/filesystem.hpp>

using namespace ssv3d;

PCLViewer::PCLViewer(const std::string& title)
{
  m_viewer = new pcl::visualization::PCLVisualizer(title);
  m_viewer->initCameraParameters();
  m_viewer->setSize(1080,720);
  m_viewer->addCoordinateSystem (0.5, "cloud", 0);
  m_viewer->setBackgroundColor(1,1,1);
  m_viewer->setCameraPosition(5.0,-5.0,5.0,0,0,0,0,0,1,0);
}

PCLViewer::~PCLViewer()
{
  delete m_viewer;
}

bool PCLViewer::IsStop() const
{
  return m_viewer->wasStopped();
}

void PCLViewer::AddPointCloud(const WSPointCloudPtr cloud, int vp)
{
  m_viewer->removeAllShapes();
  m_viewer->removeAllPointClouds();
  std::string name("cloud_");
  name.append(std::to_string(vp));
  m_viewer->addPointCloud<WSPoint>(cloud,name,vp);
  m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,name,vp);

  // set camera position
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::getMinMax3D<WSPoint>(*cloud, min, max);
  double dRadius = 1.0*std::sqrt((max[0]-min[0])*(max[0]-min[0])+(max[1]-min[1])*(max[1]-min[1])+(max[2]-min[2])*(max[2]-min[2]));
  m_viewer->setCameraPosition(dRadius,-dRadius,dRadius,centroid[0],centroid[1],centroid[2],0,0,1,vp);
}

void PCLViewer::AddCube(const WSPoint& point, double s, int id, double r,double g, double b, int vp)
{
  std::string name("cube_");
  name.append(std::to_string(id));
  m_viewer->removeShape(name);
  m_viewer->addCube(point.x-0.5*s,point.x+0.5*s,point.y-0.5*s,point.y+0.5*s,point.z-0.5*s,point.z+0.5*s,r,g,b,name,vp);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1, name);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, name);
}

void PCLViewer::AddArrow(const WSPoint& pt, const WSNormal& normal, double length, int id, double r,double g, double b, int vp)
{
  std::string name("arrow_");
  name.append(std::to_string(id));
  m_viewer->removeShape(name);
  WSPoint pte;
  pte.x = pt.x + length*normal.normal_x;
  pte.y = pt.y + length*normal.normal_y;
  pte.z = pt.z + length*normal.normal_z;
  m_viewer->addArrow(pt,pte,r,g,b,false,name,vp);
}

void PCLViewer::AddMesh(const pcl::PolygonMesh& mesh)
{
  if (!m_viewer->updatePolygonMesh(mesh, "mesh"))
  {
    m_viewer->addPolygonMesh(mesh, "mesh");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mesh");
  }
}

void PCLViewer::AddCoordinateSystem(const Eigen::Affine3f& camPose, int idIndex, double scale,int vp, bool removeall)
{
  std::string name("ccs");
  name.append(std::to_string(vp));
  name.append(std::to_string(idIndex));
  m_viewer->addCoordinateSystem(scale,camPose,name,vp);
}

void PCLViewer::SpinOnce(double duration)
{
  m_viewer->spinOnce(duration);
}

void PCLViewer::Spin() const
{
  m_viewer->spin();
}

void PCLViewer::AddText(const std::string& text, const std::string& id, int vp)
{
    m_viewer->addText(text,50,50,30,0.0,0.0,0.0,id,vp);
}

void PCLViewer::AddLine(const WSPoint& startPt, const WSPoint& endPt, int idx, double r,double g, double b, int vp)
{
  std::string id("line_");
  id.append(std::to_string(idx));
  m_viewer->addLine(startPt,endPt,r,g,b,id,vp);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id);
}

void PCLViewer::AddPolygon(const WSPointCloudPtr& polygon, const std::string& id, double r, double g, double b, int vp)
{
  m_viewer->addPolygon<WSPoint>(polygon,r,g,b,id,vp);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1, id);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, id);
  m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, id);
}

void PCLViewer::AddNormals(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal, int size, double arrow, int vp)
{
  std::string name("normal_");
  name.append(std::to_string(vp));
  m_viewer->addPointCloudNormals<WSPoint, WSNormal>(cloud,normal,size,arrow,name,vp);
}
