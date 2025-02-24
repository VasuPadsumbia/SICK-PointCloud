#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include <iostream>
#include <sstream>

#include <chrono>
#include <thread>

#include <application_base_modules/Camera.h>
#include <application_base_modules/PLCData.h>

int main(int argc, char* argv[])
{
  /// Default values:
  /// IP:        "
  std::string deviceIpAddr("192.168.1.10");
  std::string deviceType("Visionary-S");
  int         streamPort = 2114;

  Camera& camera = Camera::getInstance();
  pcl::PointCloud<pcl::PointXYZ>    contours; 
  Eigen::Vector3d                   centroid; 
  Eigen::Vector3d      point_color;
  double eps = 0.05;
  int min_samples = 15;
  //PLCData                           plc_data("localhost", 4841);
  std::shared_ptr<PLCData> plc_data        = std::make_shared<PLCData>("localhost", 4841);
  auto    init_start_time = std::chrono::high_resolution_clock::now();
  //camera.setROI(215, 535, 151, 384);
  camera.initializeStream();
  auto init_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> init_elapsed_seconds = init_end_time - init_start_time;
  std::cout << "Initialization time: " << init_elapsed_seconds.count() << "s\n";
  
  auto start_time = std::chrono::high_resolution_clock::now();
  camera.processFrame(false, false, false, false, false, false);
 
  //camera.setDepthRange(std::make_tuple(0.533, 0.538));
  camera.setDepthRange(std::make_tuple(0.5065, 0.512));
  std::tie(contours, centroid, point_color)     = camera.getContours(true, eps, min_samples);
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  std::cout << "Frame process time: " << elapsed_seconds.count() << "s\n";
  
  std::cout << "Centroid: " << centroid << std::endl;
  std::cout << "Point color: " << point_color << std::endl;
  std::cout << "Contour size: " << contours.size() << std::endl;

  auto cleanup_start_time = std::chrono::high_resolution_clock::now();
  camera.cleanup();
  auto cleanup_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> cleanup_elapsed_seconds = cleanup_end_time - cleanup_start_time;
  std::cout << "Cleanup time: " << cleanup_elapsed_seconds.count() << "s\n";
}
