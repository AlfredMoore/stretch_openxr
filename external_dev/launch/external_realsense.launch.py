import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ====================================================================
    # === IMPORTANT: SET THE PATH TO YOUR EXTERNAL EXECUTABLE HERE ===
    # ====================================================================
    # Replace this with the actual, absolute path to your application
    executable_path = '/home/vr-robot/realsenseZMQ/build/rs_zmq_publisher'
    # For example: '/usr/local/bin/my_network_app'
    # ====================================================================

    # Declare the launch arguments
    declare_serial_arg = DeclareLaunchArgument(
        'serial',
        default_value='000000000000',
        description='Serial number for the RealSense camera 12 digits.'
    )

    declare_path_arg = DeclareLaunchArgument(
        'path',
        description='Path to the external executable.'
    )

    # declare_address_arg = DeclareLaunchArgument(
    #     'address',
    #     default_value='tcp://*:5555',
    #     description='Address for the ZMQ publisher binding. E.g., tcp://*:5555'
    # )

    serial = LaunchConfiguration('serial')
    path = LaunchConfiguration('path')
    # address = LaunchConfiguration('address')
    
    run_external_app = ExecuteProcess(
        cmd=[
            path,
            '--serial', serial,
            # '--show',
            # '--address', address,
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_serial_arg,
        declare_path_arg,
        # declare_address_arg,
        run_external_app,
    ])
