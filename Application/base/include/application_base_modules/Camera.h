#ifndef CAMERA_H
#define CAMERA_H

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <chrono>
#include <thread>

#include <sick_visionary_cpp_base/CoLaParameterWriter.h>
#include <sick_visionary_cpp_base/CoLaParameterReader.h>
#include <sick_visionary_cpp_base/FrameGrabber.h>
#include <sick_visionary_cpp_base/PointCloudPlyWriter.h>
#include <sick_visionary_cpp_base/PointXYZ.h>
#include <sick_visionary_cpp_base/VisionaryControl.h>
#include <sick_visionary_cpp_base/VisionaryType.h>
//#include <vtk-9.3/vtkSmartPointer.h>
#include "exitcodes.h"
#include "framewrite.h"
#include "BlobServerConfig.h"
#include "Visualizer.h"

using namespace visionary;


    class Camera {
    public:
        static Camera& getInstance();

        void run(int count = 1, bool image_plot = false, bool point_cloud_plot = false, bool PCD = false, bool PLY = false, bool save_image = false, bool save_point_cloud = false);
        void setROI(int left_roi, int right_roi, int top_roi, int bottom_roi);
        void setIntegrationTime(int integration_time, int integration_time_color);
        void setDepthRange(std::tuple<float, float> depth_range);
        ExitCode initializeStream();
        void cleanup();
        ExitCode processFrame(bool image_plot = false, bool point_cloud_plot = false, bool PCD = false, bool PLY = false, bool save_image = false, bool save_point_cloud = false);
        std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
            Eigen::Vector3d, 
            Eigen::Vector3d> getContours(
                bool plot = false, double eps = 15, int min_samples = 10);
        std::tuple<std::vector<float>, std::vector<float>, std::tuple<int, int>> 
            getDepthRange();
        std::uint64_t  getTimestampMS() const
        {
          return timestamp_ms;
        }
    private:
        Camera();
        ~Camera();
        
        const std::string ip_address;
        std::string device_type;
        int stream_port;
        bool roi;
        int left_roi;
        int right_roi;
        int top_roi;
        int bottom_roi;
        int integration_time;
        int integration_time_color;
        std::tuple<float, float> depth_range;
        VisionaryType visionaryType = VisionaryType(VisionaryType::Enum::eVisionaryS);
        std::shared_ptr<VisionaryControl> device_control;
        std::vector<PointXYZ> point_cloud_ply;
        std::vector<float> dist_data;
        Eigen::Vector3d                   centroid;
        pcl::PointCloud<pcl::PointXYZ>    contour;
        Eigen::Vector3d      point_color;
        bool setup_updated;
        std::uint64_t  timestamp_ms;
        std::time_t timestamp;
        std::shared_ptr<VisionaryData> pDataHandler = nullptr;
        std::unique_ptr<FrameGrabberBase> pFrameGrabber = nullptr;
        Camera(const Camera&) = delete;
        Camera& operator=(const Camera&) = delete;
        unsigned pollperiodMs = 500u;
        VisionaryData* sensor_data;
        std::shared_ptr<Visualizer> visualizer;
    };

#endif // CAMERA_H
