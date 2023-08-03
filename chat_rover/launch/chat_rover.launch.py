from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='object_recognition',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='voice2text',
        output='screen'
        )
    ])