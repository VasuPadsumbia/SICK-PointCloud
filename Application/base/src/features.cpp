#include "features.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>

PointCloudProcessor::PointCloudProcessor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                         float eps,
                                         int min_samples)
  : cloud(cloud), eps(eps), min_samples(min_samples)
{
  
  for (const auto& p : cloud->points)
  {
    pcl::PointXYZ buffer;
    buffer.x = p.x;
    buffer.y = p.y;
    buffer.z = p.z;
    //std::cout << "x: " << buffer.x << ", y: " << buffer.y << ", z: " << buffer.z << std::endl;
    points.push_back(buffer);
    //std::cout << "x: " << points.back().x << ", y: " << points.back().y << ", z: " << points.back().z << std::endl;
    colors.emplace_back(p.r , p.g , p.b);
  }
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
    std::vector<Eigen::Vector3d>> PointCloudProcessor::filterPointsByDepth(
  const std::tuple<float, float>& depth_range)
{
  float min_depth, max_depth;
  std::tie(min_depth, max_depth) = depth_range;
  pcl::PointCloud<pcl::PointXYZ> filtered_points;
  std::vector<Eigen::Vector3d> filtered_colors;
  //std::cout << "Min depth in feature extraction: " << min_depth << ", Max depth in feature extraction: " << max_depth << std::endl;
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (points[i].z >= min_depth && points[i].z <= max_depth)
    {
      filtered_points.push_back(points[i]);
      //std::cout << "x: " << points[i].x << ", y: " << points[i].y << ", z: " << points[i].z << std::endl;
      filtered_colors.push_back(colors[i]);
    }
  }

  std::cout << "Filtered points count: " << filtered_points.size() << std::endl;
  return std::make_tuple(filtered_points, filtered_colors);
}

std::vector<Eigen::Vector2d> PointCloudProcessor::projectTo2D(
    const std::vector<Eigen::Vector3d>& points)
{
  std::vector<Eigen::Vector2d> projected_points;
  for (const auto& p : points)
  {
    projected_points.emplace_back(p.x(), p.y());
  }
  std::cout << "Projected points count: " << projected_points.size() << std::endl;
  return projected_points;
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
    Eigen::Vector3d> PointCloudProcessor::findContour(
  const pcl::PointCloud<pcl::PointXYZ>& points, double eps, int min_samples)
{
  if (points.empty())
  {
    std::cout << "No points to find contour." << std::endl;
    return std::make_tuple(pcl::PointCloud<pcl::PointXYZ>(), Eigen::Vector3d::Zero());
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  //for (const auto& p : points)
  //{
  //  cloud_xyz->points.emplace_back(p.x, p.y, p.z);
  //}
  // Downsample the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ>       sor;
  sor.setInputCloud(points.makeShared());
  sor.setLeafSize(0.005f, 0.005f, 0.005f); // Adjust the leaf size as needed
  sor.filter(*cloud_xyz);

  std::cout << "Downsampled points count: " << cloud_xyz->size() << std::endl;

  tree->setInputCloud(cloud_xyz);

  std::vector<pcl::PointIndices>                 cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(eps);
  ec.setMinClusterSize(min_samples);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_xyz);
  ec.extract(cluster_indices);
  std::cout << "eps: " << eps << ", min_samples: " << min_samples << std::endl;
  if (cluster_indices.empty())
  {
    std::cout << "No valid clusters found." << std::endl;
    return std::make_tuple(pcl::PointCloud<pcl::PointXYZ>(), Eigen::Vector3d::Zero());
  }

  std::vector<Eigen::Vector3d> largest_cluster_points;
    #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(cluster_indices[0].indices.size()); ++i)
  {
    int idx = cluster_indices[0].indices[i];
    #pragma omp critical
    largest_cluster_points.push_back(
      Eigen::Vector3d(cloud_xyz->points[idx].x, cloud_xyz->points[idx].y, cloud_xyz->points[idx].z));
  }

  std::vector<cv::Point2f> largest_cluster_points_2d;
    #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(largest_cluster_points.size()); ++i)
  {
    const auto& p = largest_cluster_points[i];
    #pragma omp critical
    largest_cluster_points_2d.emplace_back(p.x(), p.y());
  }

  std::vector<int> hull_indices;
  cv::convexHull(largest_cluster_points_2d, hull_indices, false);

  pcl::PointCloud<pcl::PointXYZ> hull;
    #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(hull_indices.size()); ++i)
  {
    int idx = hull_indices[i];
    #pragma omp critical
    hull.push_back(
      pcl::PointXYZ(largest_cluster_points[idx].x(), largest_cluster_points[idx].y(), largest_cluster_points[idx].z()));
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  if (!hull.empty())
  {
    Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();
    #pragma omp parallel
    {
      Eigen::Vector3d local_sum = Eigen::Vector3d::Zero();
      #pragma omp for nowait
      for (int i = 0; i < static_cast<int>(hull.size()); ++i)
      {
        const auto& p = hull.points[i];
        local_sum += Eigen::Vector3d(p.x, p.y, p.z);
      }
        #pragma omp critical
      {
        centroid_sum += local_sum;
      }
    }
    centroid = centroid_sum / static_cast<double>(hull.size());
  }

  return std::make_tuple(hull, centroid);
}



Eigen::Vector3d PointCloudProcessor::getColor(const Eigen::Vector3d& point)
{
  auto it = std::min_element(points.begin(), points.end(), [&point](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
    Eigen::Vector3d a_eigen(a.x, a.y, a.z);
    Eigen::Vector3d b_eigen(b.x, b.y, b.z);
    return (a_eigen - point).norm() < (b_eigen - point).norm();
  });

  int index = std::distance(points.begin(), it);
  return Eigen::Vector3d{colors[index]};
}

void PointCloudProcessor::visualizePointCloudWithContour(const pcl::PointCloud<pcl::PointXYZ>& contour,
                                                         const Eigen::Vector3d&              centroid)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  for (size_t i = 0; i < points.size(); ++i)
  {
    pcl::PointXYZRGB point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.r = static_cast<uint8_t>(colors[i].x() * 255);
    point.g = static_cast<uint8_t>(colors[i].y() * 255);
    point.b = static_cast<uint8_t>(colors[i].z() * 255);
    cloud->points.push_back(point);
  }

  pcl::visualization::PCLVisualizer::Ptr viewer(
    new pcl::visualization::PCLVisualizer("Point Cloud with Contour and Cursor Info"));
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
  // Add custom XYZ axes with thinner lines
  pcl::PointXYZ origin(0.0, 0.0, 0.0);
  pcl::PointXYZ x_axis(1.0, 0.0, 0.0);
  pcl::PointXYZ y_axis(0.0, 1.0, 0.0);
  pcl::PointXYZ z_axis(0.0, 0.0, 1.0);

  viewer->addLine(origin, x_axis, 1.0, 0.0, 0.0, "x_axis");
  viewer->addLine(origin, y_axis, 0.0, 1.0, 0.0, "y_axis");
  viewer->addLine(origin, z_axis, 0.0, 0.0, 1.0, "z_axis");

  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "x_axis");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "y_axis");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "z_axis");

  if (!contour.empty())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : contour)
    {
      contour_cloud->points.emplace_back(p.x, p.y, p.z);
    }
    viewer->addPolygon<pcl::PointXYZ>(contour_cloud, 1.0, 0.0, 0.0, "contour");
  }

  pcl::PointXYZ centroid_point(centroid.x(), centroid.y(), centroid.z());
  viewer->addSphere(centroid_point, 0.005, 1.0, 0.0, 0.0, "centroid");

  viewer->spin();
}
