from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    teleop_udp_node = Node(
        package='teleop_udp',  # Your package name
        executable='quest_teleop',   # The executable name from add_executable
        name='stretch_teleop_node',
        output='screen',
        parameters=[{
        'udp_port': 12345,
        'linear_scale': 0.5,
        'angular_scale': 0.5,
        }]
    )
    return LaunchDescription([
        teleop_udp_node,
    ])
