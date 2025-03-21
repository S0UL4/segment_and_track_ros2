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
        DeclareLaunchArgument('lidar_selected', default_value='1', description='Lidar Type , 0 : Livox , other : Standard Pointcloud2'),
        DeclareLaunchArgument('lidar_topic_input', default_value='/velodyne_points', description='Lidar input topic'),
        DeclareLaunchArgument('lidar_topic_output', default_value='/livox/lidar_organized', description='Lidar output topic'),
        DeclareLaunchArgument('lidar_frame', default_value='velodyne', description='Lidar frame'),
        DeclareLaunchArgument('theta_angle_degree', default_value='90.0', description='Rotation pointcloud % Z '),
        DeclareLaunchArgument('keep_side', default_value='left', description='Side to keep in lidar processing'),
        DeclareLaunchArgument('radius_filetring_compared_to_livox', default_value='0.5', description='Radius from lidar origin (float)'),
        DeclareLaunchArgument('k_neighboors_normal_estimation', default_value='1.5', description='K-neighbors for normal estimation (float)'),
        DeclareLaunchArgument('use_ground_segmentation', default_value='true', description='Enable ground segmentation (bool)'),
        DeclareLaunchArgument('ground_filter_distance_threshold', default_value='0.05', description='Ground filter distance threshold (float)'),
        DeclareLaunchArgument('ground_filter_max_iterations', default_value='1000.0', description='Ground filter max iterations (float)'),
        DeclareLaunchArgument('ground_filter_angle_threshold', default_value='10.0', description='Ground filter angle threshold (float)'),
        DeclareLaunchArgument('ground_filter_inverse_z', default_value='false', description='Ground filter inverse Z (bool)'),
        DeclareLaunchArgument('side_segementation_smoothnessThreshold', default_value='90.0', description='If the deviation between points normals is less than the smoothness threshold then they are suggested to be in the same cluster ( !! en degrees )( float )'),
        DeclareLaunchArgument('y_side_filter', default_value='20.0', description='Y filter max distance (m)'),
        DeclareLaunchArgument('x_side_min_filter', default_value='0.0', description='pointcloud at X min distance towards lidar to take into account '),
        DeclareLaunchArgument('x_side_max_filter', default_value='7.0', description='pointcloud up to X max m to take into account '),
        DeclareLaunchArgument('downsample_voxel_leaf_size', default_value='0.2', description='downsample before segmenting for faster computing ( in meter ) '),
        DeclareLaunchArgument('setMinClusterSize', default_value='200', description='setMinClusterSize for segmentation side '),
        DeclareLaunchArgument('setNumberOfNeighbours', default_value='30', description='setNumberOfNeighbours for segmentation side'),

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
                'side_segementation_smoothnessThreshold': LaunchConfiguration('side_segementation_smoothnessThreshold'),
                'lidar_selected': LaunchConfiguration('lidar_selected'),
                'y_side_filter': LaunchConfiguration('y_side_filter'),
                'x_side_min_filter' : LaunchConfiguration('x_side_min_filter'),
                'x_side_max_filter' : LaunchConfiguration('x_side_max_filter'),
                'theta_angle_degree' : LaunchConfiguration('theta_angle_degree'),
                'downsample_voxel_leaf_size' : LaunchConfiguration('downsample_voxel_leaf_size'),
                'setMinClusterSize' : LaunchConfiguration('setMinClusterSize'),
                'setNumberOfNeighbours' : LaunchConfiguration('setNumberOfNeighbours')
            }]
        ),
        Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='string_to_overlay_text_1',
            output='screen',
            parameters=[
                {"string_topic": "lisiere_detected"},
                {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
        )
    ])

