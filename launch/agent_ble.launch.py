from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='Perceptron',
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

    reconnect_delay_arg = DeclareLaunchArgument(
        'reconnect_delay',
        default_value='3',
        description='Seconds between reconnection attempts (0 to disable)'
    )

    rssi_interval_arg = DeclareLaunchArgument(
        'rssi_interval',
        default_value='5',
        description='RSSI logging interval in seconds (0 to disable)'
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
            '--reconnect-delay', LaunchConfiguration('reconnect_delay'),
            '--rssi-interval', LaunchConfiguration('rssi_interval'),
        ]
    )

    return LaunchDescription([
        device_name_arg,
        scan_timeout_arg,
        verbose_arg,
        reconnect_delay_arg,
        rssi_interval_arg,
        agent_node_device
    ])
