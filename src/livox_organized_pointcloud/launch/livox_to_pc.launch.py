import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch
################### user configure parameters for ros2 start ###################
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
################### user configure parameters for ros2 end #####################


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('lidar_topic_input', default_value='/livox/lidar', description='Lidar input topic'),
        DeclareLaunchArgument('lidar_topic_output', default_value='/livox/lidar_organized', description='Lidar output topic'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_lidar_frame', description='Lidar frame'),
        DeclareLaunchArgument('keep_side', default_value='right', description='Side to keep in lidar processing'),
        DeclareLaunchArgument('radius_filetring_compared_to_livox', default_value='0.5', description='Radius from lidar origin (float)'),
        DeclareLaunchArgument('k_neighboors_normal_estimation', default_value='1.5', description='K-neighbors for normal estimation (float)'),
        DeclareLaunchArgument('use_ground_segmentation', default_value='true', description='Enable ground segmentation (bool)'),
        DeclareLaunchArgument('ground_filter_distance_threshold', default_value='0.10', description='Ground filter distance threshold (float)'),
        DeclareLaunchArgument('ground_filter_max_iterations', default_value='1000.0', description='Ground filter max iterations (float)'),
        DeclareLaunchArgument('ground_filter_angle_threshold', default_value='10.0', description='Ground filter angle threshold (float)'),
        DeclareLaunchArgument('ground_filter_inverse_z', default_value='false', description='Ground filter inverse Z (bool)'),
        DeclareLaunchArgument('side_segementation_smoothnessThreshold', default_value='45.0', description='If the deviation between points normals is less than the smoothness threshold then they are suggested to be in the same cluster ( !! en degrees )( float )'),

        # Node execution (example, modify for your use case)
        Node(
            package='livox_organized_pointcloud',
            executable='livox_organized_pointcloud_node',
            name='livox_organized_pointcloud_node',
            output='screen',
            # prefix = ["gdbserver localhost:3000"],
            parameters=[{
                'lidar_topic_input': LaunchConfiguration('lidar_topic_input'),
                'lidar_topic_output': LaunchConfiguration('lidar_topic_output'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'keep_side': LaunchConfiguration('keep_side'),
                'radius_filetring_compared_to_livox': LaunchConfiguration('radius_filetring_compared_to_livox'),
                'k_neighboors_normal_estimation': LaunchConfiguration('k_neighboors_normal_estimation'),
                'use_ground_segmentation': LaunchConfiguration('use_ground_segmentation'),
                'ground_filter_distance_threshold': LaunchConfiguration('ground_filter_distance_threshold'),
                'ground_filter_max_iterations': LaunchConfiguration('ground_filter_max_iterations'),
                'ground_filter_angle_threshold': LaunchConfiguration('ground_filter_angle_threshold'),
                'ground_filter_inverse_z': LaunchConfiguration('ground_filter_inverse_z'),
                'side_segementation_smoothnessThreshold': LaunchConfiguration('side_segementation_smoothnessThreshold')
            }]
        )
    ])

