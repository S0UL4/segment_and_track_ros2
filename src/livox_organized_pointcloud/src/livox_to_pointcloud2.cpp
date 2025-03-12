/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Iheb Soula
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "livox_to_pointcloud2.hpp"

using namespace std::chrono_literals;

LivoxToPointCloud2::LivoxToPointCloud2() : Node("livox_to_pointcloud2")
{
    // LIDAR PARAMS
    this->declare_parameter<std::string>("lidar_topic_input", "/livox/lidar"); // defaults to "/livox/lidar"
    this->declare_parameter<std::string>("lidar_topic_output", "/livox/lidar_organized"); // defaults to "/livox/lidar"
    this->declare_parameter<std::string>("lidar_frame", "livox_lidar_frame"); // defaults to "/livox/lidar"
    // Which side of livox ?
    this->declare_parameter<std::string>("side", "right"); // defaults to "/right"

    // filter robot ( alentours )
    this->declare_parameter<float>("radius", 1.0); //Default radius = 1m



    this->get_parameter("lidar_topic_input", lidar_topic_input_);
    this->get_parameter("lidar_topic_output", lidar_topic_output_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("radius", radius_);  // Default radius = 1m
    this->get_parameter("side", side_);



    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_output_, 10);
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_input_, 10, std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));
}

void LivoxToPointCloud2::callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    // if(msg->point_num < min_points_) return;
    
    sensor_msgs::msg::PointCloud2 output;
    
    output.header = msg->header;
    output.header.frame_id=lidar_frame_;
    output.fields.resize(4);

    output.fields[0].name = "x";
    output.fields[0].offset = offsetof(pcl::PointXYZI,x);
    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[0].count = 1;

    output.fields[1].name = "y";
    output.fields[1].offset = offsetof(pcl::PointXYZI,y);
    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[1].count = 1;

    output.fields[2].name = "z";
    output.fields[2].offset = offsetof(pcl::PointXYZI,z);
    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[2].count = 1;

    output.fields[3].name = "intensity";
    output.fields[3].offset = offsetof(pcl::PointXYZI,intensity);
    output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[3].count = 1;

    output.point_step = sizeof(pcl::PointXYZI);
    output.row_step = output.point_step * msg->point_num;
    output.data.resize(output.row_step);

    uint8_t* raw_data_ptr = output.data.data();
    for (const auto& point : msg->points)
    {
        *(reinterpret_cast<float*>(raw_data_ptr + offsetof(pcl::PointXYZI,x))) = point.x;
        *(reinterpret_cast<float*>(raw_data_ptr + offsetof(pcl::PointXYZI,y))) = point.y;
        *(reinterpret_cast<float*>(raw_data_ptr + offsetof(pcl::PointXYZI,z))) = point.z;
        *(reinterpret_cast<float*>(raw_data_ptr + offsetof(pcl::PointXYZI,intensity))) = static_cast<float>(point.reflectivity);

        raw_data_ptr += output.point_step;
    }

    
    output.width = msg->point_num;
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>); // only on Y
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZI>); // Y and X

    sensor_msgs::msg::PointCloud2 output_filtered; 
    pcl::fromROSMsg(output, *pcl_cloud);
    // filter on Y
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(pcl_cloud);
    pass_y.setFilterFieldName("y");
    if(!side_.compare("right")) // if its RIGHT
        pass_y.setFilterLimits(- std::numeric_limits<float>::max(), 0);   // only negatif values 
    else
        pass_y.setFilterLimits(0, std::numeric_limits<float>::max());  // only pos values

    pass_y.filter(*cloud_filtered);

     // filter on radius
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_filtered);
    // search per rapport to origin
    pcl::PointXYZI searchPoint;
    searchPoint.x = 0.0;
    searchPoint.y = 0.0;
    searchPoint.z = 0.0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    kdtree.radiusSearch(searchPoint,radius_,pointIdxRadiusSearch,pointRadiusSquaredDistance);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    
    extract.setInputCloud(cloud_filtered);

    pcl::PointIndices::Ptr remove_indices(new pcl::PointIndices);
    remove_indices->indices = pointIdxRadiusSearch ;
    extract.setIndices(remove_indices);
    extract.setNegative (true);
    extract.filter(*cloud_filtered_final);


    pcl::toROSMsg(*cloud_filtered_final,output_filtered);

    publisher_->publish(output_filtered);
    
}