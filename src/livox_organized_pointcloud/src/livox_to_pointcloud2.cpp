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

    // Y filter 
    this->declare_parameter<float>("y_side_filter", 10.0);

    // X min and max filter ( pour que le robot voit a x_min devant )
    this->declare_parameter<float>("x_side_min_filter", 2.0);
    this->declare_parameter<float>("x_side_max_filter", 20.0);




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

    this->get_parameter("y_side_filter",y_side_filter);
    this->get_parameter("x_side_min_filter",x_side_min_filter);
    this->get_parameter("x_side_max_filter",x_side_max_filter);


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
    RCLCPP_INFO(this->get_logger(),"y_side_filter : %f",y_side_filter);
    RCLCPP_INFO(this->get_logger(),"x_side_min_filter : %f",x_side_min_filter);
    RCLCPP_INFO(this->get_logger(),"x_side_max_filter : %f",x_side_max_filter);


    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_output_, 10);
    inliers_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented",10);
    publisher_colored = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_point_cloud",10);
    centroid_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("centroid_markers", 10);

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
    if(msg->data.size() == 0) return;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>); // only on Y
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZI>); // Y and X

    sensor_msgs::msg::PointCloud2 output_filtered; 
    pcl::fromROSMsg(*msg, *pcl_cloud);
    // filter on Y
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_y.setInputCloud(pcl_cloud);
    pass_y.setFilterFieldName("y");
    if(!side_.compare("right")) // if its RIGHT
        pass_y.setFilterLimits(- y_side_filter , 0);   // only negatif values 
    else if(!side_.compare("left"))
        pass_y.setFilterLimits(0, y_side_filter);  // only pos values
    else 
        pass_y.setFilterLimits(- y_side_filter, y_side_filter);  // only pos values
  
    pass_y.filter(*cloud_filtered);

    pass_x.setInputCloud(cloud_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_side_min_filter,x_side_max_filter);
    pass_x.filter(*cloud_filtered);

    if(cloud_filtered->size() == 0 ) return;
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
    
    if(colored_cloud->size() == 0 ) return;
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
    if(msg->point_num < 50 ) return;
    
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZI>); // Y and X

    sensor_msgs::msg::PointCloud2 output_filtered; 
    pcl::fromROSMsg(output, *pcl_cloud);
    // filter on Y
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_y.setInputCloud(pcl_cloud);
    pass_y.setFilterFieldName("y");
    if(!side_.compare("right")) // if its RIGHT
        pass_y.setFilterLimits(- y_side_filter , 0);   // only negatif values 
    else if(!side_.compare("left"))
        pass_y.setFilterLimits(0, y_side_filter);  // only pos values
    else 
        pass_y.setFilterLimits(- y_side_filter, y_side_filter);  // only pos values
  
    pass_y.filter(*cloud_filtered);

    pass_x.setInputCloud(cloud_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_side_min_filter,x_side_max_filter);
    pass_x.filter(*cloud_filtered);

    if(cloud_filtered->size() == 0 ) return;
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
    
    if(colored_cloud->size() == 0 ) return;
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
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50);
    reg.setInputCloud (pc_in);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (side_segementation_smoothnessThreshold_ / 180.0 * M_PI);
    reg.setCurvatureThreshold (3.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    // Step 1: Calculate centroids
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZI>);
    visualization_msgs::msg::MarkerArray marker_array;
    int i=0;

    if(clusters.size () == 0 )
    {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr empty(new pcl::PointCloud <pcl::PointXYZRGB>());
        return empty;

    }
    

    for (const auto& cluster : clusters) {
        Eigen::Vector3f centroid(0.0, 0.0, 0.0);
        for (auto idx : cluster.indices) {
            centroid += pc_in->points[idx].getVector3fMap();
        }
        centroid /= static_cast<float>(cluster.indices.size());
        centroids->push_back(pcl::PointXYZI(centroid.x(), centroid.y(), centroid.z()));

        // Create centroid marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = lidar_frame_;  // Use the correct frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "centroids";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid.x();
        marker.pose.position.y = centroid.y();
        marker.pose.position.z = centroid.z();
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration(100ms);

        marker_array.markers.push_back(marker);
        i+=1;


    }





    centroid_marker_pub_->publish(marker_array);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(centroids);

    std::vector<int> merged(clusters.size(), -1);  // Cluster merging labels
    float merge_distance = 1.0;  // 1-meter threshold

    // Step 3: Merge clusters based on Kd-tree neighbor search ( yum yum yum )
    for (size_t i = 0; i < centroids->size(); ++i) {
        if (merged[i] != -1) continue;  // Already merged
        merged[i] = i;

        std::vector<int> nearby_indices;
        std::vector<float> distances;
        kdtree.radiusSearch(centroids->points[i], merge_distance, nearby_indices, distances);

        for (auto idx : nearby_indices) {
            if (merged[idx] == -1) merged[idx] = i;
        }
    }

    // garder le plus proche
    // int K = 1;
    // std::vector<int> pointIdxKNNSearch(K);
    // std::vector<float> pointKNNSquaredDistance(K);
    
    // pcl::PointXYZI searchPoint;

    // searchPoint.x = 0.0;
    // searchPoint.y = 0.0;
    // searchPoint.z = 0.0;
    
    // if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    //   {
    //    for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
    //      std::cout << "    "  <<   (*centroids)[ pointIdxKNNSearch[i] ].x 
    //                << " " << (*centroids)[ pointIdxKNNSearch[i] ].y 
    //                << " " << (*centroids)[ pointIdxKNNSearch[i] ].z 
    //                << " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;
    //  }


    //  marker_array.markers.clear();


    //  visualization_msgs::msg::Marker marker;
    //  marker.header.frame_id = lidar_frame_;  // Use the correct frame
    //  marker.header.stamp = this->get_clock()->now();
    //  marker.ns = "centroids";
    //  marker.id = i;
    //  marker.type = visualization_msgs::msg::Marker::SPHERE;
    //  marker.action = visualization_msgs::msg::Marker::ADD;
    //  marker.pose.position.x = (*centroids)[ pointIdxKNNSearch[i] ].x ;
    //  marker.pose.position.y = (*centroids)[ pointIdxKNNSearch[i] ].y 
    //  marker.pose.position.z = (*centroids)[ pointIdxKNNSearch[i] ].z 
    //  marker.scale.x = 1.0;
    //  marker.scale.y = 1.0;
    //  marker.scale.z = 1.0;
    //  marker.color.a = 1.0;
    //  marker.color.r = 1.0;
    //  marker.color.g = 0.0;
    //  marker.color.b = 0.0;
    //  marker.lifetime = rclcpp::Duration(100ms);



    //  marker_array.markers.push_back(marker);


    





    // // Step 4: Create merged cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < clusters.size(); ++i) {
        int cluster_id = merged[i];
        uint8_t r = (cluster_id * 193) % 256;
        uint8_t g = (cluster_id * 97) % 256;
        uint8_t b = (cluster_id * 53) % 256;

        for (auto idx : clusters[i].indices) {
            pcl::PointXYZRGB point;
            point.x = pc_in->points[idx].x;
            point.y = pc_in->points[idx].y;
            point.z = pc_in->points[idx].z;
            point.r = r;
            point.g = g;
            point.b = b;
            merged_cloud->points.push_back(point);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Merged cloud has %lu points", merged_cloud->size());
    merged_cloud->width = merged_cloud->size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    return merged_cloud;
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