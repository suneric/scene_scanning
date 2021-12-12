#include <vector>
#include "process.h"
#include "filter.h"
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

using namespace ssv3d;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <WSPointNormal>
{
  using pcl::PointRepresentation<WSPointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const WSPointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

PCLProcess::PCLProcess()
{
  m_cloud = WSPointCloudPtr(new WSPointCloud);
  m_globalTransform = Eigen::Matrix4f::Identity();
  m_cloudList.clear();
  m_viewpointList.clear();
}

PCLProcess::~PCLProcess()
{
}

bool PCLProcess::Registration()
{
  size_t vpCount = m_viewpointList.size();
  size_t ptCount = m_cloudList.size();
  if (vpCount == 0 || ptCount == 0 || vpCount != ptCount)
    return false;

  std::cout << "data count" << ptCount << '\n';

  // filter point cloud
  PCLFilter filter;
  WSPointCloudPtr cloud = m_cloudList[ptCount-1];
  cloud = filter.FilterPCLPoint(cloud, 0.01);

  // transform point cloud to global frame
  std::vector<double> vpMat = m_viewpointList[ptCount-1];
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      trans(i,j) = vpMat[i*4+j];

  WSPointCloudPtr tCloud (new WSPointCloud);
  pcl::transformPointCloud (*cloud, *tCloud, trans);
  // merge point cloud to global point cloud;

  // filter pt in z
  tCloud = filter.FilterPassThrough(tCloud,"z",0.05,10);
  *m_cloud += *tCloud;

  m_viewpointList.pop_back();
  m_cloudList.pop_back();

  return true;
}

WSPointCloudPtr PCLProcess::PointCloud() const
{
  return m_cloud;
}

void PCLProcess::AddViewPoint(const std::vector<double>& vp)
{
  m_viewpointList.push_back(vp);
}
void PCLProcess::AddPointCloud(const WSPointCloudPtr& cloud)
{
  m_cloudList.push_back(cloud);
}


void PCLProcess::PairAlign (const WSPointCloudPtr cloud_src,
  const WSPointCloudPtr cloud_tgt,
  WSPointCloudPtr output,
  Eigen::Matrix4f &final_transform,
  bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  WSPointCloudPtr src(new WSPointCloud);
  WSPointCloudPtr tgt(new WSPointCloud);
  pcl::VoxelGrid<WSPoint> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  //
  // Compute surface normals and curvature
  WSPointCloudWithNormalsPtr points_with_normals_src(new WSPointCloudWithNormals);
  WSPointCloudWithNormalsPtr points_with_normals_tgt(new WSPointCloudWithNormals);

  pcl::NormalEstimation<WSPoint, WSPointNormal> norm_est;
  pcl::search::KdTree<WSPoint>::Ptr tree (new pcl::search::KdTree<WSPoint> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<WSPointNormal, WSPointNormal> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  WSPointCloudWithNormalsPtr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }
