from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='chat_rover',
        executable='gpt1_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='gpt2_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='qr_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='vosk_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='yolo_node',
        output='log'
        ),
        Node(
        package='chat_rover_bt',
        executable='main_node',
        output='screen'
        )
    ])