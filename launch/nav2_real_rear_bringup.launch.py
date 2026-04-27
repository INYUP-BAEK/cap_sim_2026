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

    # 🚨 1. 파일 경로 설정 (SLAM 툴박스 파일 삭제, 2D 맵 파일 추가)
    params_file = os.path.join(cap_sim_dir, 'config', 'nav2_real_rear_params.yaml') # STVL이 적용된 Nav2 파라미터
    map_file = '/home/inyup/colcon_ws/src/cap_sim_2026/maps/320.yaml' # 👈 아까 파이썬으로 뽑아낸 2D 도면 절대경로!
    rviz_config_file = os.path.join(cap_sim_dir, 'rviz', 'my.rviz')

    # 실제 로봇 구동이므로 False로 설정합니다.
    use_sim_time = 'False' 

    # 🚨 2. Map Server 노드 (SLAM Toolbox를 대신하여 2D 도면을 띄워주는 역할)
    map_server_node = Node( 
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file,
                     'use_sim_time': False}]
    )

    # Map Server 활성화 관리자 (Map Server를 준비 상태에서 활성 상태로 켜주는 필수 노드)
    map_server_lifecyle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    # 🚨 3. Nav2 코어 실행 (amcl 제외, 순수 navigation 알고리즘만 실행)
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    # 4. RViz2 실행
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    
    # 🚨 실행 순서대로 추가 (맵을 먼저 깔고 -> Nav2 알고리즘을 올립니다)
    ld.add_action(map_server_node)           # 2D 맵 로드
    ld.add_action(map_server_lifecyle_node)  # 2D 맵 퍼블리시 시작
    ld.add_action(nav2_launch_cmd)           # 경로 계획 & STVL 실행
    ld.add_action(rviz_cmd)

    return ld