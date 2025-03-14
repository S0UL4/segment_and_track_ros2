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
    this->declare_parameter<std::string>("keep_side", "all"); // defaults to "all"

    // filter robot ( alentours )
    this->declare_parameter<float>("radius_filetring_compared_to_livox", 0.5); //Default radius = 50 cm ( sphere of 50 cm)
    this->declare_parameter<float>("k_neighboors_normal_estimation", 1.0); //Default radius Sphere = 1 m

    // ground filtering ? maybe useful also 
    this->declare_parameter<bool>("use_ground_segmentation",true);
    this->declare_parameter<float>("ground_filter_distance_threshold", 0.10); //Default radius = 10 cm 
    this->declare_parameter<float>("ground_filter_max_iterations", 1000.0); //Default to 1000
    this->declare_parameter<float>("ground_filter_angle_threshold", 10.0);
    this->declare_parameter<bool>("ground_filter_inverse_z", false);

    // side segmentation
    this->declare_parameter<float>("side_segementation_smoothnessThreshold", 45.0);  // en degrees

    // selected lidar 
    this->declare_parameter<int>("lidar_selected",0); // 0 : Livox , 1 : Standard Pointcloud

    this->get_parameter("lidar_topic_input", lidar_topic_input_);
    this->get_parameter("lidar_topic_output", lidar_topic_output_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("radius_filetring_compared_to_livox", radius_);  // Default radius = 1m
    this->get_parameter("keep_side", side_);
    this->get_parameter("k_neighboors_normal_estimation", k_neighboors_normal_estimation_);

    // ground params
    this->get_parameter("use_ground_segmentation", use_ground_segmentation_);
    this->get_parameter("ground_filter_distance_threshold", ground_filter_distance_threshold_);
    this->get_parameter("ground_filter_max_iterations", ground_filter_max_iterations_);
    this->get_parameter("ground_filter_angle_threshold", ground_filter_angle_threshold_);
    this->get_parameter("ground_filter_inverse_z", inverse_z_);
    this->get_parameter("side_segementation_smoothnessThreshold", side_segementation_smoothnessThreshold_);
    this->get_parameter("lidar_selected", lidar_selected_);


    RCLCPP_INFO(this->get_logger(),"Node Init with : \n");
    RCLCPP_INFO(this->get_logger()," lidar_selected : %i",lidar_selected_);
    RCLCPP_INFO(this->get_logger()," lidar_topic_input : %s",lidar_topic_input_.c_str());
    RCLCPP_INFO(this->get_logger()," lidar_topic_output : %s",lidar_topic_output_.c_str());
    RCLCPP_INFO(this->get_logger()," lidar_frame : %s",lidar_frame_.c_str());
    RCLCPP_INFO(this->get_logger()," radius_filetring_compared_to_livox : %f",radius_);
    RCLCPP_INFO(this->get_logger()," Side to keep of pointcloud : %s",side_.c_str());
    RCLCPP_INFO(this->get_logger()," k_neighboors_normal_estimation : %f",k_neighboors_normal_estimation_);
    RCLCPP_INFO(this->get_logger()," use_ground_segmentation : %i",use_ground_segmentation_);
    RCLCPP_INFO(this->get_logger()," ground_filter_distance_threshold : %f",ground_filter_distance_threshold_);
    RCLCPP_INFO(this->get_logger()," ground_filter_max_iterations : %f",ground_filter_max_iterations_);
    RCLCPP_INFO(this->get_logger()," ground_filter_angle_threshold : %f",ground_filter_angle_threshold_);
    RCLCPP_INFO(this->get_logger(),"ground_filter_inverse_z : %i",inverse_z_);
    RCLCPP_INFO(this->get_logger(),"side_segementation_smoothnessThreshold : %f",side_segementation_smoothnessThreshold_);


    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_output_, 10);
    inliers_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented",10);
    publisher_colored = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_point_cloud",10);

    if(lidar_selected_ == 0)
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lidar_topic_input_, 10, std::bind(&LivoxToPointCloud2::callback_livox, this, std::placeholders::_1));
    else 
        subscription_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_input_, 10, std::bind(&LivoxToPointCloud2::callback_pc, this, std::placeholders::_1));
  
    }


void LivoxToPointCloud2::upsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud) {
        double search_radius = 0.01;
        double sampling_radius = 0.001;
        double step_size =0.001;
        double gauss_param = (double)std::pow(search_radius, 2);
        int pol_order = 2;
        unsigned int num_threats = 1;

        // https://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html
        // check alternative https://pointclouds.org/documentation/classpcl_1_1_bilateral_upsampling.html
        pcl::PointCloud<pcl::PointXYZI>::Ptr dense_points(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
        pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;

        mls.setComputeNormals(true);
        mls.setInputCloud(input_cloud);
        mls.setSearchMethod(kd_tree);
        mls.setSearchRadius(search_radius);
        mls.setUpsamplingMethod(
        pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(sampling_radius);
        mls.setUpsamplingStepSize(step_size);
        mls.setPolynomialOrder(pol_order);
        mls.setSqrGaussParam(gauss_param);  // (the square of the search radius works best in general)
        mls.setCacheMLSResults(true);       // Set whether the mls results should be stored for each point in the input cloud.
        mls.setNumberOfThreads(num_threats);
        // mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method
        // mls.setPointDensity(15); //15
        mls.process(*dense_points);

        *output_cloud = *input_cloud;
        *output_cloud += *dense_points;

        if (output_cloud->points.size() == input_cloud->points.size()) {
            return;
        //pcl::console::print_warn("\ninput cloud could not be upsampled, change input parameters!");
        }

        RCLCPP_INFO(this->get_logger(),"New points %d ",dense_points->points.size());
        RCLCPP_INFO(this->get_logger(),"Output cloud points %d ",output_cloud->points.size());
}


void LivoxToPointCloud2::callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>); // only on Y
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZI>); // Y and X

    sensor_msgs::msg::PointCloud2 output_filtered; 
    pcl::fromROSMsg(*msg, *pcl_cloud);
    // filter on Y
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(pcl_cloud);
    pass_y.setFilterFieldName("y");
    if(!side_.compare("right")) // if its RIGHT
        pass_y.setFilterLimits(- std::numeric_limits<float>::max(), 0);   // only negatif values 
    else if(!side_.compare("left"))
        pass_y.setFilterLimits(0, std::numeric_limits<float>::max());  // only pos values
    else 
        pass_y.setFilterLimits(- std::numeric_limits<float>::max(), std::numeric_limits<float>::max());  // only pos values
  
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

    // Take points from the ground ( pour le moment estimation RANSAC , plan fit )
    if(use_ground_segmentation_)
    {
        pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr Inliers_ground(new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointClouds = extractPlanes(cloud_filtered_final, Inliers_ground, coefficients_ground, ground_filter_distance_threshold_, ground_filter_angle_threshold_, ground_filter_max_iterations_);
    
        cloud_filtered_final = pointClouds ;
    }

    // upsampling maybe ??? 
    pcl::PointCloud<pcl::PointXYZI>::Ptr up_sample_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    upsampling(cloud_filtered_final,up_sample_filtered);
    pcl::toROSMsg(*up_sample_filtered,output_filtered);
    output_filtered.header = msg->header;
    output_filtered.header.frame_id = lidar_frame_;
    output_filtered.is_dense = true;
    publisher_->publish(output_filtered);

    // Segmentation based on REGION GROWING algorithm 
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = segment_side(cloud_filtered_final,k_neighboors_normal_estimation_);
    
    // Publish Colored Cloud
    sensor_msgs::msg::PointCloud2 colored_cloud_ros;
    pcl::toROSMsg(*colored_cloud,colored_cloud_ros);
    colored_cloud_ros.header = msg->header;
    colored_cloud_ros.header.frame_id = lidar_frame_;
    colored_cloud_ros.is_dense = false;
    publisher_colored->publish(colored_cloud_ros);

}


void LivoxToPointCloud2::callback_livox(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
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
    output.is_dense = false;

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
    else if(!side_.compare("left"))
        pass_y.setFilterLimits(0, std::numeric_limits<float>::max());  // only pos values
    else 
        pass_y.setFilterLimits(- std::numeric_limits<float>::max(), std::numeric_limits<float>::max());  // only pos values
  
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

    // Take points from the ground ( pour le moment estimation RANSAC , plan fit )
    if(use_ground_segmentation_)
    {
        pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr Inliers_ground(new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointClouds = extractPlanes(cloud_filtered_final, Inliers_ground, coefficients_ground, ground_filter_distance_threshold_, ground_filter_angle_threshold_, ground_filter_max_iterations_);
    
        cloud_filtered_final = pointClouds ;
    }


    // upsampling maybe ??? 
    pcl::PointCloud<pcl::PointXYZI>::Ptr up_sample_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    upsampling(cloud_filtered_final,up_sample_filtered);
    pcl::toROSMsg(*up_sample_filtered,output_filtered);
    output_filtered.header = msg->header;
    output_filtered.header.frame_id = lidar_frame_;
    output_filtered.is_dense = true;
    publisher_->publish(output_filtered);


    // Segmentation based on REGION GROWING algorithm 
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = segment_side(up_sample_filtered,k_neighboors_normal_estimation_);
    
    // Publish Colored Cloud
    sensor_msgs::msg::PointCloud2 colored_cloud_ros;
    pcl::toROSMsg(*colored_cloud,colored_cloud_ros);
    colored_cloud_ros.header = msg->header;
    colored_cloud_ros.header.frame_id = lidar_frame_;
    colored_cloud_ros.is_dense = true;
    publisher_colored->publish(colored_cloud_ros);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr LivoxToPointCloud2::segment_side(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in, float k_neighboors)
{
    // Segmentation based on REGION GROWING algorithm 

    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimator; // maybe keep it one thread instead of NormalEstimationOMP ? ( i mean NormalEstimation )
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (pc_in);
    normal_estimator.setRadiusSearch(k_neighboors); // performe ( k neighborhood)
    normal_estimator.compute (*normals);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*pc_in, *indices);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (200);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (pc_in);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (side_segementation_smoothnessThreshold_ / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    RCLCPP_INFO(this->get_logger(),"Number of clusters is equal to %i", clusters.size ());
    RCLCPP_INFO(this->get_logger(),"First cluster has %i", clusters[0].indices.size () , " points." );

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();


    return colored_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr LivoxToPointCloud2::extractPlanes(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, float distanceThreshold, float angle, int maxIterations)
{
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMaxIterations(maxIterations);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);

    //because we want a specific plane (X-Y Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0); 
    if(inverse_z_)
        axis = Eigen::Vector3f(0.0, 0.0, - 1.0); //z axis
    seg.setAxis(axis);
    seg.setEpsAngle(angle * (M_PI / 180.0f)); // plane can be within angle degrees of X-Y plane

    // Create pointcloud to publish inliers
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr inversed_cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());

    // Fit a plane
    seg.setInputCloud(pc);
    seg.segment(*inliers, *coefficients);

    // Check result
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        // break;
    }

    // Extract inliers
    extract.setInputCloud(pc);
    extract.setIndices(inliers);
    extract.setNegative(false);
    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    // Let only the points that are not in the planar surface
    extract.setNegative(true);
    extract.filter(*inversed_cloud_plane);

    return inversed_cloud_plane;
}