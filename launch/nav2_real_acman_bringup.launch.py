import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 찾기
    cap_sim_dir = get_package_share_directory('cap_sim_2026')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 🚨 1. 파일 경로 설정 (가장 중요한 변경점: 아커만 전용 파라미터 파일 로드!)
    params_file = os.path.join(cap_sim_dir, 'config', 'nav2_real_acman_params.yaml') 
    map_file = '/home/inyup/colcon_ws/src/cap_sim_2026/maps/320.yaml' 
    rviz_config_file = os.path.join(cap_sim_dir, 'rviz', 'my.rviz')

    # 실제 로봇 구동
    use_sim_time_bool = False 
    use_sim_time_str = 'False'

    # 2. Map Server 노드 (2D 도면 로드)
    map_server_node = Node( 
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file,
                     'use_sim_time': use_sim_time_bool}]
    )

    # 3. Map Server Lifecycle 매니저
    map_server_lifecyle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_bool,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    # 🚨 4. Nav2 코어 실행 (아커만 파라미터를 주입하여 실행)
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'params_file': params_file, # 👈 여기가 아커만 모델을 결정짓습니다.
            'autostart': 'True'
        }.items()
    )

    # 5. RViz2 실행
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