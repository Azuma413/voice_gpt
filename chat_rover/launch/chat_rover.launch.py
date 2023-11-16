from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen'
        ),
        #Node(
        #package='v4l2_camera',
        #executable='v4l2_camera_node',
        #output='screen'
        #),
        Node(
        package='chat_rover',
        executable='gpt1_node',
        output='screen'
        ),
        Node(
        package='chat_rover',
        executable='gpt2_node',
        output='screen'
        ),
        Node(
        package='chat_rover',
        executable='ar_node',
        output='screen'
        ),
        Node(
        package='chat_rover',
        executable='vosk_node',
        output='log'
        ),
        Node(
        package='chat_rover',
        executable='yolo_node',
        output='screen'
        ),
        Node(
        package='chat_rover_bt',
        executable='pos2vel_node',
        output='screen'
        ),
        #Node(
        #package='chat_rover_bt',
        #executable='main_node',
        #output='log'
        #)
    ])