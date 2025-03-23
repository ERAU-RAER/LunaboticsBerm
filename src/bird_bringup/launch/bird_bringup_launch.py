import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

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
        package='g2si',
        executable='g2si_node',
        name='g2si_node',
        output='screen',
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
            ('/imu/data', '/imu/data_raw_livox')
        ]
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'),'launch','foxglove_bridge_launch.xml')
        )
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(get_package_share_directory('bird_description'),'urdf', 'bird.urdf.xml')).read()}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    madgwick_lidar = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_lidar_node',
        output='screen',
        remappings=[
            ('/imu/data_raw', '/imu/data_raw_livox'),
            ('/imu/data', '/imu/data_livox')
        ],
        parameters=[os.path.join(get_package_share_directory('bird_bringup'), 'params', 'madgwick_lidar.yaml')]
    )

    robot_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('bird_bringup'), 'params', 'ekf.yaml')]
    )

    return LaunchDescription([
        agent_lidar_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        g2si_node,
        madgwick_lidar,
        robot_local_node,
        foxglove_launch
    ])