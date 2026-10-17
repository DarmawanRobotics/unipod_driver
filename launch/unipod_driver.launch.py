"""UniPod MT11 driver: PTZ, zoom, video streams, thermal, recenter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('unipod_driver'), 'config', 'unipod_config.yaml')

    args = [
        DeclareLaunchArgument('namespace', default_value='unipod'),
        DeclareLaunchArgument('config', default_value=config,
                              description='parameter file'),
        DeclareLaunchArgument('transport', default_value='tcp',
                              description='tcp, udp or serial'),
        DeclareLaunchArgument('ip', default_value='10.21.31.26',
                              description='pod IP (factory 192.168.144.25)'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
    ]

    driver = Node(
        package='unipod_driver',
        executable='unipod_driver_node',
        name='unipod_driver',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'transport': LaunchConfiguration('transport'),
                'ip': LaunchConfiguration('ip'),
                'serial_device': LaunchConfiguration('serial_device'),
            },
        ],
    )

    return LaunchDescription(args + [driver])
