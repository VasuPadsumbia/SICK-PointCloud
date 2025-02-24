#ifndef VISUAL_H
#define VISUAL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <sick_visionary_cpp_base/VisionaryControl.h>
#include <sick_visionary_cpp_base/VisionaryData.h>
#include <sick_visionary_cpp_base/VisionaryType.h>
#include <sick_visionary_cpp_base/VisionarySData.h>
#include <sick_visionary_cpp_base/PointXYZ.h>
#include <pcl/visualization/point_picking_event.h>

using namespace visionary;

class Visualizer
{
public:
  Visualizer(const std::vector<PointXYZ>&   point_cloud = {},
             const std::vector<uint32_t>&    rgbaMap     = {},
             const std::tuple<float, float>& depth_range = {0.0f, 0.0f});

    std::tuple<pcl::PointCloud<pcl::PointXYZ>, 
        Eigen::Vector3d, 
        Eigen::Vector3d> 
        processAndVisualizePointCloud(
            bool visualize = false,
            double eps         = 15.0f,
            int   min_samples = 10);
    void visualizePointCloud();
    void plotFrameRealTime(const VisionaryData& data);
    void setPointCloudColor(
        const std::vector<PointXYZ>& point_cloud, 
        const std::vector<uint32_t>& rgbaMap);
    void setDepthRange(const std::tuple<float, float>& depth_range);
    
  private:
    std::vector<PointXYZ> point_cloud; // Change reference to value
    std::vector<uint32_t> rgbaMap; // Change reference to value
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb;
    std::string type;
    std::tuple<float, float> depth_range;
    std::vector<Eigen::Vector3d> pick_points;
    Eigen::Vector3d           centroid;
    pcl::PointCloud<pcl::PointXYZ>         contour;
    Eigen::Vector3d           point_colors;

  private:
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
};

#endif // VISUAL_H




