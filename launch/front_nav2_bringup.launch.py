import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def generate_launch_description():
    package_name = 'cap_sim_2026'
    namespace = 'front'

    my_pkg_dir = get_package_share_directory(package_name)
    params_file = os.path.join(my_pkg_dir, 'config', 'nav2_params_front.yaml')
    
    # 프론트봇 전용 SLAM Toolbox 파라미터 파일 경로
    slam_params_file = os.path.join(my_pkg_dir, 'config', 'front_slam_toolbox.yaml')

    ld = LaunchDescription()

    # 모든 노드를 'front' 네임스페이스로 묶고 실행하는 그룹
    front_nav_group = GroupAction(actions=[
        PushRosNamespace(namespace),

        # 전역 토픽 연결 (TF 및 Map 공유)
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        SetRemap(src='map', dst='/map'),

        # 1. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # 2. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),

        # 3. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),

        # 4. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),

        # 5. Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),         # 원본 명령 수신
                ('cmd_vel_smoothed', 'cmd_vel')     # 최종 명령 송신
            ]
        ),

        # 6. SLAM Toolbox (로컬라이제이션 모드)
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            # 🚨 YAML 버그 방지를 위해 필수 프레임 정보를 런치에서 "강제 주입"
            parameters=[
                slam_params_file, 
                {
                    'use_sim_time': True,
                    'odom_frame': 'front/odom',
                    'base_frame': 'front/base_footprint',
                    'scan_topic': '/front/scan'
                }
            ],
            remappings=[
                ('map', '/front/dummy_map'), 
                ('/map', '/front/dummy_map'), 
                ('tf', '/tf'), 
                ('/tf', '/tf'), 
                ('tf_static', '/tf_static'),
                ('/tf_static', '/tf_static')
            ]
        ),

        # 7. 통합 라이프사이클 매니저
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_front',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother'  # 🚨 핵심: 스무더를 매니저 명단에 반드시 추가!!
                ]
            }]
        )
    ])

    ld.add_action(front_nav_group)

    return ld