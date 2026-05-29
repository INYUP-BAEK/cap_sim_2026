import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('cap_sim_2026')
    default_config = os.path.join(
        share_dir,
        'config',
        'front2rear_domain_bridge_allowlist.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value=default_config,
                description='domain_bridge YAML config file.',
            ),
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
            Node(
                package='domain_bridge',
                executable='domain_bridge',
                name='front2rear_domain_bridge',
                output='screen',
                arguments=[
                    '--wait-for-publisher',
                    'true',
                    '--wait-for-subscription',
                    'false',
                    LaunchConfiguration('config'),
                ],
            ),
        ]
    )
