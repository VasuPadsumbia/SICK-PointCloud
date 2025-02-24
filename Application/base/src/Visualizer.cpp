#include "Visualizer.h"
#include "features.h"
//#include <vtk-9.3/vtkSmartPointer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <numeric>
#include <tuple> // Add this include for std::tie

Visualizer::Visualizer(const std::vector<PointXYZ>&  point_cloud,
                       const std::vector<uint32_t>&    rgbaMap,
                       const std::tuple<float, float>& depth_range)
  : point_cloud(point_cloud), rgbaMap(rgbaMap), depth_range(depth_range)
{
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
    Eigen::Vector3d, Eigen::Vector3d>
  Visualizer::processAndVisualizePointCloud(
        bool  visualize,
        double eps,
        int   min_samples)
{
  std::unique_ptr<PointCloudProcessor> feature = std::make_unique<PointCloudProcessor>(this->pcl_rgb, eps, min_samples);
  pcl::PointCloud<pcl::PointXYZ>       filtered_points;
  std::vector<Eigen::Vector3d> filtered_colors;
  //std::cout << "Depth range: " << std::get<0>(this->depth_range) << ", " << std::get<1>(this->depth_range) << std::endl;
  std::tie(filtered_points, filtered_colors) = feature->filterPointsByDepth(this->depth_range);
  std::tie(this->contour, this->centroid) = feature->findContour(filtered_points, eps, min_samples);
  this->point_colors = feature->getColor(this->centroid);
  if (visualize)
  {
    feature->visualizePointCloudWithContour(this->contour, this->centroid);
  }
  return std::make_tuple(this->contour, this->centroid, this->point_colors);
}

void Visualizer::visualizePointCloud()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Point Cloud"));
    viewer->addPointCloud<pcl::PointXYZRGB>(pcl_rgb, "sample cloud");

    // Set point size
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    // Add XYZ axes in red, green, and blue colors
    viewer->addCoordinateSystem(1.0, "coordinate system");

    // Set thinner lines for the XYZ axes
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "coordinate system");

    // Set up mouse event handler to display coordinates
    viewer->registerPointPickingCallback(&Visualizer::pointPickingCallback, *this);

    if (!this->contour.empty())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr contour_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : this->contour)
        {
            pcl::PointXYZ point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            contour_cloud->points.push_back(point);
        }
        viewer->addPolygon<pcl::PointXYZ>(contour_cloud, 1.0, 0.0, 0.0, "contour");
    }

    pcl::PointXYZ centroid_point(this->centroid.x(), this->centroid.y(), this->centroid.z());
    viewer->addSphere(centroid_point, 0.01, 0.6, 0.0, 0.0, "centroid");
    // Add a point with x and y set to zero and z representing any value
    pcl::PointXYZ zero_xy_point(0.0, 0.0, this->centroid.z());
    viewer->addSphere(zero_xy_point, 0.001, 1.0, 0.0, 0.0, "zero_xy_point");

    // Add XY plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0; // a
    coefficients->values[1] = 1.0; // b
    coefficients->values[2] = 0.0; // c
    coefficients->values[3] = 0.0; // d
    viewer->addPlane(*coefficients, "xy_plane");

    viewer->spin();
}

void Visualizer::plotFrameRealTime(const VisionaryData& data)
{
    // Get the width and height of the images
  const std::uint32_t width  = data.getWidth();
    const std::uint32_t height = data.getHeight();
  const VisionarySData& rVisionarySData = dynamic_cast<const VisionarySData&>(data);
      
  std::vector<std::uint32_t> rgba_data  = rVisionarySData.getRGBAMap();
  std::vector<std::uint16_t> zmap_data  = rVisionarySData.getZMap();
  std::vector<std::uint16_t> state_data = rVisionarySData.getStateMap();

    // Create OpenCV matrices
  cv::Mat rgba_display(height, width, CV_8UC4);
  cv::Mat zmap_display(height, width, CV_16UC1);
  cv::Mat state_display(height, width, CV_16UC1);

  // Copy data to OpenCV matrices
  std::memcpy(rgba_display.data, rgba_data.data(), rgba_data.size() * sizeof(std::uint32_t));
  std::memcpy(zmap_display.data, zmap_data.data(), zmap_data.size() * sizeof(std::uint16_t));
  std::memcpy(state_display.data, state_data.data(), state_data.size() * sizeof(std::uint16_t));

  // Normalize ZMap and StateMap for display
  cv::normalize(zmap_display, zmap_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::normalize(state_display, state_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  // Display the images
  cv::imshow("RGBA Map", rgba_display);
  cv::imshow("Z-Map", zmap_display);
  cv::imshow("State Map", state_display);
  cv::waitKey(0);
}

void Visualizer::setPointCloudColor(
    const std::vector<PointXYZ>& point_cloud, 
    const std::vector<uint32_t>& rgbaMap)
{
  if (point_cloud.size() != rgbaMap.size())
  {
    std::cerr << "Error: Point cloud and color map sizes do not match." << std::endl;
    return;
  }
  if (this != nullptr)
  {
    pcl_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < point_cloud.size(); ++i)
    {
      pcl::PointXYZRGB point;
      point.x         = point_cloud[i].x;
      point.y         = point_cloud[i].y;
      point.z         = point_cloud[i].z;
      const auto rgba = reinterpret_cast<const uint8_t*>(&rgbaMap[i]);
      point.r         = rgba[0];
      point.g         = rgba[1];
      point.b         = rgba[2];
      pcl_rgb->points.push_back(point);
      //if (point.r != 0 || point.g != 0 || point.b != 0)
      //{
      //  std::cout << "Point: " << point.x << ", " << point.y << ", " << point.z << std::endl;
      //  std::cout << "Color: " << static_cast<int>(point.r) << ", " << static_cast<int>(point.g) << ", "
      //            << static_cast<int>(point.b) << std::endl;
      //}
      //if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
      //{
      //  std::cout << "NaN Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
      //  std::cout << "Color: (" << static_cast<int>(point.r) << ", " << static_cast<int>(point.g) << ", "
      //            << static_cast<int>(point.b) << ")" << std::endl;
      //}
    }
  }
  
}

void Visualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  if (event.getPointIndex() != -1)
  {
    pcl::PointXYZRGB point = pcl_rgb->points[event.getPointIndex()];
    std::cout << "Point ( " << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    std::cout << "Point color ( " << static_cast<int>(point.r) << ", " << static_cast<int>(point.g) << ", "
              << static_cast<int>(point.b) << ")" << std::endl;
  }
}
// tag::visualizer[]

void Visualizer::setDepthRange(const std::tuple<float, float>& depth_range)
{
  this->depth_range = depth_range;
}


