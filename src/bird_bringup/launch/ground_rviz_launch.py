import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')

def generate_launch_description():
    urdf_model_path = os.path.join(
        get_package_share_directory('bird_description'),
        'urdf', 
        'bird.urdf.xml')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,

    ])

if __name__ == '__main__':
    generate_launch_description()