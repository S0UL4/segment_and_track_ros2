import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
################### user configure parameters for ros2 start ###################
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    livox_organized_pc = Node(
        package='livox_organized_pointcloud',
        executable='livox_organized_pointcloud_node',
        name='livox_organized_pointcloud_node',
        output='screen',
        # prefix = ["gdbserver localhost:3000"],
        parameters=[
                {"lidar_topic_input": "/livox/lidar"},
                {"lidar_topic_output": "/livox/lidar_organized"},
                {"lidar_frame": "livox_lidar_frame"},
                 {"side": "left"},
                 {"radius": 0.5}
            ])

    return LaunchDescription([
        livox_organized_pc,
    ])
