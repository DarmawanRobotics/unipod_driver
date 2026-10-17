"""UniPod MT11: driver (PTZ, zoom, video mode, thermal, recenter) and both RTSP streams."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import yaml


def node_params(config, node_path):
    """Merge the ros__parameters sections of `config` that match `node_path`.

    Keys are matched by their trailing tokens ('/**/camera' matches 'unipod/main/camera',
    '/**/main/camera' only the main one; more specific sections win). Foxy's rcl does not
    expand '/**/name' patterns in parameter files, so the launch file does it.
    """
    path = [t for t in node_path.split('/') if t]
    sections = []
    for key, value in (config or {}).items():
        tokens = [t for t in str(key).split('/') if t and t != '**']
        if not tokens or tokens == path[len(path) - len(tokens):]:
            sections.append((len(tokens), (value or {}).get('ros__parameters', {})))
    merged = {}
    for _, params in sorted(sections, key=lambda s: s[0]):
        merged.update(params)
    return merged


def setup(context):
    get = context.launch_configurations.get
    with open(get('config')) as f:
        config = yaml.safe_load(f)
    ns = get('namespace').strip('/')
    ip = get('ip')

    driver_params = node_params(config, f'{ns}/unipod_driver')
    driver_params.update({
        'transport': get('transport'),
        'ip': ip,
        'serial_device': get('serial_device'),
    })
    actions = [Node(
        package='unipod_driver',
        executable='unipod_driver_node',
        name='unipod_driver',
        namespace=ns,
        output='screen',
        parameters=[driver_params],
    )]

    if get('camera').lower() in ('true', '1', 'yes'):
        for stream, path in (('main', 'video1'), ('sub', 'video2')):
            params = {'frame_id': f'unipod_{stream}_optical_frame'}
            params.update(node_params(config, f'{ns}/{stream}/camera'))
            params['url'] = f'rtsp://{ip}:8554/{path}'
            actions.append(Node(
                package='unipod_driver',
                executable='unipod_camera_node',
                name='camera',
                namespace=f'{ns}/{stream}',
                output='screen',
                parameters=[params],
            ))
    return actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('unipod_driver'), 'config', 'unipod_config.yaml')

    return LaunchDescription([
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
        OpaqueFunction(function=setup),
    ])
