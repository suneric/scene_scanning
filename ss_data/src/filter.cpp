#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/sampling_surface_normal.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "filter.h"

using namespace ssv3d;

WSPointCloudPtr PCLFilter::FilterPCLPointInBBox(const WSPointCloudPtr cloud, const std::vector<double>& bbox, bool bFilter)
{
  if (cloud == nullptr)
    return nullptr;

  pcl::CropBox<WSPoint> boxFilter(true);
  boxFilter.setMin(Eigen::Vector4f(bbox[0], bbox[1], bbox[2], 1.0));
  boxFilter.setMax(Eigen::Vector4f(bbox[3], bbox[4], bbox[5], 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.setNegative(bFilter);
  std::vector<int> indices;
  boxFilter.filter(indices);
  // std::cout << indices.size() << " " << bFilter << std::endl;
  return ExtractPoints(cloud,indices);
}

WSPointCloudPtr PCLFilter::FilterPassThrough(const WSPointCloudPtr cloud, const std::string& field, double limit_min, double limit_max)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr passCloud(new WSPointCloud());
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(limit_min, limit_max);
  // pass.setFilterLimitsNegative(true);
  pass.filter(*passCloud);
  return passCloud;
}

WSPointCloudPtr PCLFilter::FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr voxelCloud(new WSPointCloud());
  pcl::VoxelGrid<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leafSize, leafSize, leafSize);
  sor.filter(*voxelCloud);
  return voxelCloud;
}

WSPointCloudPtr PCLFilter::FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr sorCloud(new WSPointCloud());
  pcl::StatisticalOutlierRemoval<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(neighbor);
  sor.setStddevMulThresh(thresh);
  sor.filter(*sorCloud);
  return sorCloud;
}

WSPointCloudPtr PCLFilter::SamplingSurfaceNormal(const WSPointCloudPtr cloud, unsigned int sample, float ratio, WSPointCloudNormalPtr& normals)
{
  if (cloud == nullptr)
    return nullptr;

  std::cout << "input cloud: " << std::to_string(cloud->points.size())<< std::endl;

  WSNormalPointCloudPtr normalCloud(new WSNormalPointCloud);
  normalCloud->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    normalCloud->points[i].x = cloud->points[i].x;
    normalCloud->points[i].y = cloud->points[i].y;
    normalCloud->points[i].z = cloud->points[i].z;
    normalCloud->points[i].r = cloud->points[i].r;
    normalCloud->points[i].g = cloud->points[i].g;
    normalCloud->points[i].b = cloud->points[i].b;
  }

  WSNormalPointCloudPtr ssnCloud(new WSNormalPointCloud);
  pcl::SamplingSurfaceNormal<WSNormalPoint> ssn;
  ssn.setInputCloud(normalCloud);
  ssn.setSample(sample);
  ssn.setRatio(ratio);
  ssn.filter(*ssnCloud);
  std::cout << "sampling surface normal: " << std::to_string(ssnCloud->points.size())<< std::endl;

  WSPointCloudPtr resCloud(new WSPointCloud);
  resCloud->points.resize(ssnCloud->points.size());
  normals->points.resize(ssnCloud->points.size());
  for (size_t i = 0; i < ssnCloud->points.size(); ++i)
  {
    resCloud->points[i].x = ssnCloud->points[i].x;
    resCloud->points[i].y = ssnCloud->points[i].y;
    resCloud->points[i].z = ssnCloud->points[i].z;
    resCloud->points[i].r = ssnCloud->points[i].r;
    resCloud->points[i].g = ssnCloud->points[i].g;
    resCloud->points[i].b = ssnCloud->points[i].b;
    normals->points[i].curvature = ssnCloud->points[i].curvature;
    normals->points[i].normal_x = ssnCloud->points[i].normal_x;
    normals->points[i].normal_y = ssnCloud->points[i].normal_y;
    normals->points[i].normal_z = ssnCloud->points[i].normal_z;
  }

  return resCloud;
}

bool PCLFilter::RandomSampling(const WSPointCloudPtr cloud, unsigned int sample, std::vector<int>& indices)
{
  if (cloud == nullptr)
    return false;

  WSPointCloudPtr rsCloud(new WSPointCloud());
  pcl::RandomSample<WSPoint> rs;
  rs.setInputCloud(cloud);
  rs.setSample(sample);
  rs.filter(indices);
  return true;
}

WSPointCloudPtr PCLFilter::UniformSampling(const WSPointCloudPtr cloud, double radius)
{
  if (cloud == nullptr)
    return nullptr;

  // WSPointCloudPtr usCloud(new WSPointCloud());
  // pcl::UniformSampling<WSPoint> us;
  // us.setInputCloud(cloud);
  // us.setRadiusSearch(radius);
  // us.filter(*usCloud);
  // return usCloud;
}

WSPointCloudPtr PCLFilter::ExtractPoints(const WSPointCloudPtr cloud, const std::vector<int>& indices)
{
  if (cloud == nullptr)
    return nullptr;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  inliers->indices = indices;
  WSPointCloudPtr sCloud(new WSPointCloud());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*sCloud);
  return sCloud;
}

WSPointCloudNormalPtr PCLFilter::ExtractNormals(const WSPointCloudNormalPtr normal, const std::vector<int>& indices)
{
  if (normal == nullptr)
    return nullptr;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  inliers->indices = indices;
  WSPointCloudNormalPtr sNormal(new WSPointCloudNormal);
  pcl::ExtractIndices<pcl::Normal> extract;
  extract.setInputCloud(normal);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*sNormal);
  return sNormal;
}

WSPointCloudPtr PCLFilter::SlicePoints(const WSPointCloudPtr cloud, const Eigen::Vector3f& rootPt, const Eigen::Vector3f& normal)
{
  if (cloud == nullptr)
    return nullptr;
  double tol = 0.005;
  std::vector<WSPoint> pts;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    WSPoint pt = cloud->points[i];
    Eigen::Vector3f cpt(pt.x, pt.y, pt.z);
    Eigen::Vector3f refNormal = (cpt - rootPt).normalized();
    double v = refNormal.dot(normal);
    if (v <= tol && v >= -tol)
    {
      // std::cout << "normal " << normal.x() << " " << normal.y() << " " << normal.z();
      // std::cout << " refnormal " <<  refNormal.x() << " " << refNormal.y() << " " << refNormal.z();
      // std::cout << " dot product " << v << std::endl;
      pts.push_back(pt);
    }
  }
  std::cout << "slicing the point coud with plane " << rootPt.x() << " " << rootPt.y() << " " << rootPt.z() << " points: " << pts.size() << std::endl;

  WSPointCloudPtr extract(new WSPointCloud);
  extract->points.resize(pts.size());
  for (int i = 0; i < pts.size(); ++i)
    extract->points[i] = pts[i];

  return extract;
}

WSPointCloudPtr PCLFilter::SortPointsInZ(const WSPointCloudPtr cloud, double resolution, double min, double max)
{
  double minz = max, maxz = min;
  WSPoint firstPt, lastPt;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    WSPoint pt = cloud->points[i];
    if (pt.z < minz)
    {
      minz = pt.z;
      firstPt = pt;
    }
    if (pt.z > maxz)
    {
      maxz = pt.z;
      lastPt = pt;
    }
  }

  std::vector<WSPoint> pts;
  WSPoint pt = firstPt;
  while (pt.z < maxz)
  {
    pts.push_back(pt);
    WSPointCloudPtr neighborPts = NeighborPoints(pt,cloud,resolution);
    size_t numPt = neighborPts->points.size();
    bool bNext = false;
    for (size_t i = 0; i < numPt; ++i)
    {
      WSPoint nextPt = neighborPts->points[i];
      if (nextPt.z > pt.z)
      {
        bNext = true;
        pt = nextPt;
      }
    }
    if (bNext == false)
      break;
  }
  pts.push_back(lastPt);

  std::cout << "sort points in Z " << pts.size() << std::endl;
  WSPointCloudPtr newCloud(new WSPointCloud);
  newCloud->points.resize(pts.size());
  for (int i = 0; i < pts.size(); ++i)
  {
    newCloud->points[i] = pts[i];
  }
  return newCloud;
}

WSPointCloudPtr PCLFilter::NeighborPoints(const WSPoint& pt, const WSPointCloudPtr cloud, double dRadius)
{
  std::vector<WSPoint> pts;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    WSPoint pt1 = cloud->points[i];
    double dist = (pt.x-pt1.x)*(pt.x-pt1.x)+(pt.y-pt1.y)*(pt.y-pt1.y)+(pt.z-pt1.z)*(pt.z-pt1.z);
    if (dist < dRadius*dRadius)
    {
      pts.push_back(pt1);
    }
  }
  WSPointCloudPtr newCloud(new WSPointCloud);
  newCloud->points.resize(pts.size());
  for (int i = 0; i < pts.size(); ++i)
  {
    newCloud->points[i] = pts[i];
  }
  return newCloud;
}

WSNormal PCLFilter::PointNormal(const WSPoint& pt, const WSPointCloudPtr cloud, double dRadius, const Eigen::Vector3f& refNormal)
{
  WSPointCloudNormalPtr normals(new WSPointCloudNormal);
  WSPointCloudPtr neighbor = NeighborPoints(pt,cloud,dRadius);
  pcl::NormalEstimation<WSPoint, WSNormal> ne;
  pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(neighbor);
  ne.setKSearch(10);
  ne.compute(*normals);
  // find average normal for the voxel
  Eigen::Vector3f vec(0,0,0);
  size_t num = normals->points.size();
  for (size_t i = 0; i < num; ++i)
  {
    WSNormal normal = normals->points[i];
    vec += Eigen::Vector3f(normal.normal_x, normal.normal_y, normal.normal_z);
  }

  Eigen::Vector3f nm = (vec/num).normalized();
  if (nm.dot(refNormal) < 0.0) // normal angle > M_PI/2
    nm = -nm;

  WSNormal normal;
  normal.normal_x = nm.x();
  normal.normal_y = nm.y();
  normal.normal_z = nm.z();
  return normal;
}
