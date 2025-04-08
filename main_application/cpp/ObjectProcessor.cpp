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

int main(int argc, char* argv[])
{
	char response_vis;
	char response_centroid;
	char response_depth;
	char response_default;

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
  std::cout << "Do you want to set to default configuration? (y/n): ";
  std::cin >> response_default;
  //while (true) {
	auto    init_start_time = std::chrono::high_resolution_clock::now();
	//camera.setROI(215, 535, 151, 384);
	camera.setIntegrationTime(1250, 6000);
	camera.initializeStream();
	auto init_end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> init_elapsed_seconds = init_end_time - init_start_time;
	std::cout << "Initialization time: " << init_elapsed_seconds.count() << "s\n";
	

	if (response_default == 'y')
	{
		  response_vis = 'n';
		  response_centroid = 'y';
		  response_depth = 'n';
	}
	else
	{
		  // Ask the user if they want to calibrate the depth range
		  std::cout << "Do you want to visualize and pick points for the depth data? (y/n): ";
		  std::cin >> response_vis;
		  std::cout << "Do you want to get centroid and point color? (y/n): ";
		  std::cin >> response_centroid;
		  std::cout << "Do you want to set the depth range? (y/n): ";
		  std::cin >> response_depth;
	}
	
	auto start_time = std::chrono::high_resolution_clock::now();
	if (response_vis == 'y')
	{
		  camera.processFrame(false, true, false, false, false, false);
	}
	else
	{
		  camera.processFrame(false, false, false, false, false, false);
	}
	
	if (response_centroid == 'y')
	{
		  if (response_depth == 'y')
		  {
			  std::cout << "Enter the minimum depth: ";
			  float min_depth;
			  std::cin >> min_depth;
			  std::cout << "Enter the maximum depth: ";
			  float max_depth;
			  std::cin >> max_depth;
			  camera.setDepthRange(std::make_tuple(min_depth, max_depth));
		  }
		  else
		  {
			  //camera.setDepthRange(std::make_tuple(0.533, 0.538));
			  camera.setDepthRange(std::make_tuple(0.530, 0.536));
			  //camera.setDepthRange(std::make_tuple(0.554, 0.569));
		  }
		  std::tie(contours, centroid, point_color) = camera.getContours(true, eps, min_samples);
		  std::cout << "Centroid: " << centroid << std::endl;
		  std::cout << "Point color: " << point_color << std::endl;
		  std::cout << "Contour size: " << contours.size() << std::endl;
	}
	
	auto                          end_time        = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = end_time - start_time;
	std::cout << "Frame process time: " << elapsed_seconds.count() << "s\n";  
	//}
  auto cleanup_start_time = std::chrono::high_resolution_clock::now();
  camera.cleanup();
  auto cleanup_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> cleanup_elapsed_seconds = cleanup_end_time - cleanup_start_time;
  std::cout << "Cleanup time: " << cleanup_elapsed_seconds.count() << "s\n";

}
