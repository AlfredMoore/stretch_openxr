from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='12345',  # Default value as a string
        description='UDP port for teleoperation'
    )
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale',
        default_value='0.5',
        description='Scaling factor for linear velocity'
    )
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale',
        default_value='0.5',
        description='Scaling factor for angular velocity'
    )

    # Node configuration
    teleop_udp_node = Node(
        package='teleop_udp',  # Your package name
        executable='quest_teleop',   # The executable name from add_executable
        name='stretch_teleop_node',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),  # Convert to int
            'linear_scale': ParameterValue(LaunchConfiguration('linear_scale'), value_type=float),  # Convert to float
            'angular_scale': ParameterValue(LaunchConfiguration('angular_scale'), value_type=float),  # Convert to float
        }]
    )

    return LaunchDescription([
        port_arg,
        linear_scale_arg,
        angular_scale_arg,
        teleop_udp_node,
    ])
