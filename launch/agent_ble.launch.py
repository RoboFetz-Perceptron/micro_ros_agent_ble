from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='ESP32-BLE',
        description='BLE device name to connect to'
    )

    scan_timeout_arg = DeclareLaunchArgument(
        'scan_timeout',
        default_value='5000',
        description='BLE scan timeout in milliseconds'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='4',
        description='Verbosity level (0-6)'
    )

    agent_node_device = Node(
        package='micro_ros_agent_ble',
        executable='micro_ros_agent_ble',
        name='micro_ros_agent_ble',
        output='screen',
        arguments=[
            '--dev', LaunchConfiguration('device_name'),
            '--timeout', LaunchConfiguration('scan_timeout'),
            '--verbose', LaunchConfiguration('verbose'),
        ]
    )

    return LaunchDescription([
        device_name_arg,
        scan_timeout_arg,
        verbose_arg,
        agent_node_device
    ])
