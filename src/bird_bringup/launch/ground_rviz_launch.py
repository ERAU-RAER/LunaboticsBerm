import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_rviz_path = cur_path + '../rviz'
rviz_config_path = os.path.join(cur_rviz_path, 'display_point_cloud_ROS2.rviz')

def generate_launch_description():

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    return LaunchDescription([
        rviz_node,

    ])

if __name__ == '__main__':
    generate_launch_description()