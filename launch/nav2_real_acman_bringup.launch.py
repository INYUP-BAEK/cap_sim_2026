import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    cap_sim_dir = get_package_share_directory('cap_sim_2026')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(cap_sim_dir, 'config', 'nav2_real_acman_params_combine.yaml') 
    map_file = '/home/baek/colcon_ws/src/cap_sim_2026/maps/320.yaml' 
    rviz_config_file = os.path.join(cap_sim_dir, 'rviz', 'my.rviz')

    use_sim_time_bool = False 
    use_sim_time_str = 'False'

    map_server_node = Node( 
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file,
                     'use_sim_time': use_sim_time_bool}]
    )

    map_server_lifecyle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_bool,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time_bool}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(map_server_node)           
    ld.add_action(map_server_lifecyle_node)  
    ld.add_action(nav2_launch_cmd)           
    ld.add_action(rviz_cmd)

    return ld