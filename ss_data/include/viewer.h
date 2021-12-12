#ifndef _SSDATA_PCL_VIEWER_H_
#define _SSDATA_PCL_VIEWER_H_

#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

typedef pcl::Normal WSNormal;
typedef pcl::PointCloud<WSNormal> WSPointCloudNormal;
typedef WSPointCloudNormal::Ptr WSPointCloudNormalPtr;

typedef pcl::PointNormal WSPointNormal;
typedef pcl::PointCloud<WSPointNormal> WSPointCloudWithNormals;
typedef WSPointCloudWithNormals::Ptr WSPointCloudWithNormalsPtr;

typedef pcl::PointXYZRGBNormal WSNormalPoint;
typedef pcl::PointCloud<WSNormalPoint> WSNormalPointCloud;
typedef WSNormalPointCloud::Ptr WSNormalPointCloudPtr;


namespace ssv3d
{
  class PCLViewer
  {
  public:
    PCLViewer(const std::string& title);
    ~PCLViewer();

    void AddPointCloud(const WSPointCloudPtr cloud, int viewport=0);
    void AddNormals(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal, int size, double arrow, int viewport=0);
    void AddMesh(const pcl::PolygonMesh& mesh);
    void AddArrows(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal, double length, int viewport=0);
    void AddCoordinateSystem(const Eigen::Affine3f& camPose, int idIndex, double scale=1.0, int viewport=0, bool removeall=false);
    void AddCube(const WSPoint& point, double s, const std::string& id, double r,double g, double b, int viewport=0);
    void AddText(const std::string& text, const std::string& id, int viewport=0);
    void AddLine(const WSPoint& startPt, const WSPoint& endPt, int idx, double r,double g, double b, int viewport=0);
    void AddPolygon(const WSPointCloudPtr& polygon, const std::string& id, double r, double g, double b, int viewport=0);

    bool IsStop() const;
    void SpinOnce(double duration = 1);
    void Spin() const;

  private:
    pcl::visualization::PCLVisualizer* m_viewer;
    WSPointCloudPtr m_ptCloud;
  };

};

#endif //!_SSDATA_PCL_VIEWER_H_
