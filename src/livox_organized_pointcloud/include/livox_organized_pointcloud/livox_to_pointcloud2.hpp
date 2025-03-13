#ifndef LIVOX_TO_POINTCLOUD2_HPP
#define LIVOX_TO_POINTCLOUD2_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <pcl/common/centroid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl/filters/passthrough.h>
// #include <pcl_ros/transforms.h>
// use kd-tree for better search of nearby
#include <pcl-1.12/pcl/kdtree/kdtree_flann.h>
// to extract indices from pcl 
#include <pcl/filters/extract_indices.h>

// for segmentation purposes ( seg with ransac )
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

// seg with region growing
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h> // to calculate normals of points
#include <pcl/search/kdtree.h> // search based on kdtree
#include <pcl/search/search.h> // search lib of pcl
#include <pcl/features/normal_3d_omp.h>

#include <vector>

namespace pcl
{
struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;                // This adds the XYZ coordinates and padding
    float intensity;                // Intensity of reflection
    std::uint16_t ring;             // Laser ring index
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
};                                  // Force SSE alignment
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZI,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2();


private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_colored;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_publisher_;
    std::string lidar_topic_input_;
    std::string lidar_topic_output_;
    std::string lidar_frame_;
    std::string side_;
    float radius_;
    float k_neighboors_normal_estimation_;
};

#endif  // LIVOX_TO_POINTCLOUD2_HPP
