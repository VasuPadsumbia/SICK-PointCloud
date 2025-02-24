#ifndef FEATURES_H
#define FEATURES_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <sick_visionary_cpp_base/PointXYZ.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor
{
public:
  PointCloudProcessor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float eps, int min_samples);
  
  std::tuple<pcl::PointCloud<pcl::PointXYZ>, std::vector<Eigen::Vector3d>> filterPointsByDepth(
    const std::tuple<float, float>& depth_range);
  
  std::vector<Eigen::Vector2d> projectTo2D(const std::vector<Eigen::Vector3d>& points);
  
  std::tuple<pcl::PointCloud<pcl::PointXYZ>, Eigen::Vector3d> findContour(const pcl::PointCloud<pcl::PointXYZ>& points,
                                                                        double                               eps,
                                                                        int min_samples);
  std::vector<std::vector<Eigen::Vector3d>> extractObjectsAndCenters();
  Eigen::Vector3d getColor(const Eigen::Vector3d& point);
  void visualizePointCloudWithContour(const pcl::PointCloud<pcl::PointXYZ>& contour, const Eigen::Vector3d& centroid);

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<Eigen::Vector3d>           colors;
  float                                  eps;
  int                                    min_samples;
};

#endif // FEATURES_H
