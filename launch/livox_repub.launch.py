from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    livox_repub_node = Node(
        package='livox_repub_ros2',
        executable='livox_repub_node',
        name='livox_repub_node',
        output='screen'
    )
    
    return LaunchDescription([
        livox_repub_node,
    ])
