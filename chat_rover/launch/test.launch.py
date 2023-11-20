from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='ar_node',
        output='screen'
        ),
        Node(
        package='chat_rover_bt',
        executable='pos2vel_node',
        output='screen'
        )
    ])