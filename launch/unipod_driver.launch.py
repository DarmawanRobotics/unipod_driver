import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('unipod_driver')
    config_file = os.path.join(pkg_dir, 'config', 'unipod_config.yaml')
    
    # Declare launch arguments
    connection_type_arg = DeclareLaunchArgument(
        'connection_type',
        default_value='tcp',
        description='Connection type: tcp, udp, or serial'
    )
    
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.21.31.26',
        description='IP address of the gimbal'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='37260',
        description='Port number for TCP/UDP connection'
    )
    
    serial_device_arg = DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyUSB0',
        description='Serial device path'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    
    # Create the node
    unipod_driver_node = Node(
        package='unipod_driver',
        executable='unipod_driver_node',
        name='unipod_driver_node',
        output='screen',
        parameters=[{
            'connection_type': LaunchConfiguration('connection_type'),
            'ip_address': LaunchConfiguration('ip_address'),
            'port': LaunchConfiguration('port'),
            'serial_device': LaunchConfiguration('serial_device'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
        }],
        remappings=[
            ('~/gimbal_attitude', '/unipod/gimbal_attitude'),
            ('~/gimbal_status', '/unipod/gimbal_status'),
            ('~/camera_status', '/unipod/camera_status'),
            ('~/thermal_data', '/unipod/thermal_data'),
            ('~/laser_data', '/unipod/laser_data'),
            ('~/ai_tracking', '/unipod/ai_tracking'),
            ('~/firmware_version', '/unipod/firmware_version'),
        ]
    )
    
    return LaunchDescription([
        connection_type_arg,
        ip_address_arg,
        port_arg,
        serial_device_arg,
        serial_baudrate_arg,
        unipod_driver_node,
    ])
