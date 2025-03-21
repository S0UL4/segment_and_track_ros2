#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav_msgs/msg/path.hpp>


using namespace std;

using namespace std::chrono_literals;

class CentroidTracker : public rclcpp::Node {
    public:
    CentroidTracker() : Node("trajectory_control") {

        marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "centroid_markers", 10, std::bind(&CentroidTracker::marker_callback, this, std::placeholders::_1));    
        publisher_goal_to_reach_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("goals",10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }
    private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_goal_to_reach_;
    std::vector<geometry_msgs::msg::Pose> projected_trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    const double safe_distance_ = 2.0;
    const double avoidance_offset_ = 1.0;  // Lateral offset for avoidance
    const double max_linear_speed_ = 0.5;
    const double max_angular_speed_ = 2.0;


    nav_msgs::msg::Path path_;
    void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      projected_trajectory_.clear();
      int id=0;
      // Process the received MarkerArray message
      RCLCPP_INFO(this->get_logger(), "Received MarkerArray with %zu markers.", msg->markers.size());
  
      // Example processing: Modify the markers in the array or use them for visualization
      visualization_msgs::msg::Marker marker_goal;
      visualization_msgs::msg::MarkerArray marker_array;
      for (auto & marker : msg->markers)
      {
        geometry_msgs::msg::Pose projected_pose;

        marker_goal.header.frame_id = "velodyne";  // Use the correct frame
        marker_goal.header.stamp = this->get_clock()->now();
        marker_goal.ns = "goals";
        marker_goal.id = id;
        marker_goal.type = visualization_msgs::msg::Marker::SPHERE;
        marker_goal.action = visualization_msgs::msg::Marker::ADD;
        marker_goal.pose.position.x = marker.pose.position.x + 0.1;
        marker_goal.pose.position.y = 0.0 ;
        marker_goal.pose.position.z = 0.0 ;
        marker_goal.scale.x = 1.0;
        marker_goal.scale.y = 1.0;
        marker_goal.scale.z = 1.0;
        marker_goal.color.a = 1.0;
        marker_goal.color.r = 0.0;
        marker_goal.color.g = 1.0;
        marker_goal.color.b = 0.0;
        marker_goal.lifetime = rclcpp::Duration(100ms);

        double distance_from_centroid =  sqrt(marker.pose.position.y * marker.pose.position.y + marker.pose.position.z * marker.pose.position.z );
        RCLCPP_INFO(this->get_logger(),"distance is : %f",distance_from_centroid);

        if(distance_from_centroid < safe_distance_ )
        {
          marker_goal.pose.position.y += avoidance_offset_ * (marker.pose.position.y  >= 0 ? 1 : -1) ;

        }
        else if(distance_from_centroid > safe_distance_ + 1.0 )
        {
          marker_goal.pose.position.y -= avoidance_offset_ * (marker.pose.position.y  >= 0 ? 1 : -1) ;
        }
        projected_pose.position.x = marker_goal.pose.position.x ;
        projected_pose.position.y = marker_goal.pose.position.y ; 

        projected_trajectory_.push_back(projected_pose);
        marker_array.markers.push_back(marker_goal);
        id+=1;

        RCLCPP_INFO(this->get_logger(), " position x = %f y = %f z = %f ",marker.pose.position.x,marker.pose.position.y,marker.pose.position.z);
      }
      publisher_goal_to_reach_->publish(marker_array);
      followProjectedTrajectory();
    }


    void followProjectedTrajectory() {
      if (projected_trajectory_.empty()) return;

      geometry_msgs::msg::Twist cmd;
      auto target = projected_trajectory_.front();

      double target_distance = std::sqrt(target.position.x * target.position.x + target.position.y * target.position.y);
      double target_angle = std::atan2(target.position.y, target.position.x);

      // Control logic
      cmd.linear.x = std::min(max_linear_speed_, 0.5 * target_distance);
      cmd.angular.z = - std::max(-max_angular_speed_, std::min(max_angular_speed_, 1.0 * target_angle));
      RCLCPP_INFO(this->get_logger(), " cmd.linear.x = %f   cmd.angular.z = %f",cmd.linear.x ,cmd.angular.z);
      cmd_pub_->publish(cmd);
  }

    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentroidTracker>());
    rclcpp::shutdown();
    return 0;
}