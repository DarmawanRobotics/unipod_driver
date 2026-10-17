"""UniPod MT11: driver (PTZ, zoom, video mode, thermal, recenter) and both RTSP streams."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('unipod_driver'), 'config', 'unipod_config.yaml')
    ns = LaunchConfiguration('namespace')
    ip = LaunchConfiguration('ip')

    args = [
        DeclareLaunchArgument('namespace', default_value='unipod'),
        DeclareLaunchArgument('config', default_value=config,
                              description='parameter file'),
        DeclareLaunchArgument('transport', default_value='tcp',
                              description='tcp, udp or serial'),
        DeclareLaunchArgument('ip', default_value='10.21.31.26',
                              description='pod IP (factory 192.168.144.25)'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='publish both RTSP streams as images'),
    ]

    driver = Node(
        package='unipod_driver',
        executable='unipod_driver_node',
        name='unipod_driver',
        namespace=ns,
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'transport': LaunchConfiguration('transport'),
                'ip': ip,
                'serial_device': LaunchConfiguration('serial_device'),
            },
        ],
    )

    def camera(stream, path, frame_id):
        return Node(
            package='unipod_driver',
            executable='unipod_camera_node',
            name='camera',
            namespace=[ns, '/', stream],
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera')),
            parameters=[
                LaunchConfiguration('config'),
                {'url': ['rtsp://', ip, ':8554/', path], 'frame_id': frame_id},
            ],
        )

    return LaunchDescription(args + [
        driver,
        camera('main', 'video1', 'unipod_main_optical_frame'),
        camera('sub', 'video2', 'unipod_sub_optical_frame'),
    ])
