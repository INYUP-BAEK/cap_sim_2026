import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 찾기
    cap_sim_dir = get_package_share_directory('cap_sim_2026')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') # 🚨 SLAM 툴박스 경로 추가

    # 파일 경로 설정
    # (주의: 맵 경로는 이제 slam_toolbox_loc.yaml 안에서 관리하므로 여기서 지웁니다)
    params_file = os.path.join(cap_sim_dir, 'config', 'nav2_rear_params.yaml')
    slam_params_file = os.path.join(cap_sim_dir, 'config', 'slam_toolbox_rear_local.yaml') # 🚨 SLAM 설정 파일 추가
    rviz_config_file = os.path.join(cap_sim_dir, 'rviz', 'my.rviz')

    # 1. RTX 최신 그래픽카드 RViz 렌더링 충돌 방지 환경변수 자동 적용
    # env_var = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')

    # 🚨 2. SLAM Toolbox 실행 (AMCL과 Map Server의 완벽한 대체자)
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'True'
        }.items()
    )

    # 🚨 3. Nav2 코어 실행 (amcl, map_server를 제외한 'navigation_launch.py'만 실행!)
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py') # 👈 여기가 핵심 변경점입니다.
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    # 4. RViz2 실행 (저장해둔 세팅 파일 적용)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    
    # 실행 순서대로 추가
    # ld.add_action(env_var)
    ld.add_action(slam_toolbox_cmd) # 👈 뼈대(TF)와 맵을 먼저 깔아주고
    ld.add_action(nav2_launch_cmd)  # 👈 주행 알고리즘을 켭니다
    ld.add_action(rviz_cmd)

    return ld
