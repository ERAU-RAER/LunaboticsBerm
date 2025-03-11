import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')

def generate_launch_description():
    agent_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bird_bringup'), 'launch', 'agent_lidar_launch.py')
        )
    )

    g2si_node = Node(
        package='g2si_package',
        executable='g2si_node',
        name='g2si_node',
        output='screen',
        remappings=[('/imu/data_raw','/livox/imu')]
    )

    return LaunchDescription([
        agent_lidar_launch,
        g2si_node
    ])