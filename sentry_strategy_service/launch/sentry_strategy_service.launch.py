import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = launch.LaunchDescription()

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('robot_bring_up'),
            'config',
            'sentry.yaml'))
    declare_yaml_path = DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='Full path to the configuration file to load'
    )
    sentry_strategy_service = Node(
        name='sentry_strategy_service',
        namespace='',
        package='sentry_strategy_service',
        executable='sentry_strategy_service_node',
        output='screen',
        parameters=[param_dir],
        respawn=True # 重启
    )

    ld.add_action(declare_yaml_path)
    ld.add_action(sentry_strategy_service)

    return ld