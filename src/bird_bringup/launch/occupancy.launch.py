from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="/cropped_cloud"),
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("topic")},
                {'position_x': 0.0},
                {'position_y': 0.0},
                {'verbose1': False},
                {'verbose2': False},
                {'cell_size': 0.05},
                {'length_x': 10.0},
                {'length_y': 10.0},
                {'frame_out': 'occupancy_map'},
                {'mapi_topic_name': 'intensity_grid'},
                {'maph_topic_name': 'height_grid'},
            ]
        )

    ])

