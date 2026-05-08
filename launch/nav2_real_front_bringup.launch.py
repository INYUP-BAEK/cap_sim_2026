import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def generate_launch_description():
    package_name = 'cap_sim_2026'
    namespace = 'front'

    my_pkg_dir = get_package_share_directory(package_name)
    params_file = os.path.join(my_pkg_dir, 'config', 'nav2_real_front_params.yaml')
    ekf_params_file = os.path.join(my_pkg_dir, 'config', 'nav2_real_front_ekf.yaml')
    
    # 🚨 [수정] 하드코딩된 경로 대신 패키지 디렉토리를 기준으로 동적 할당
    rviz_config_file = os.path.join(my_pkg_dir, 'rviz', 'front.rviz')
    map_yaml_file = os.path.join(my_pkg_dir, 'maps', '320.yaml')

    ld = LaunchDescription()

    # 🚨 [추가] RViz2 그래픽 크래시(exit code -11) 방지를 위한 소프트웨어 렌더링 강제 설정
    set_gl_env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    ld.add_action(set_gl_env)

    # 모든 노드를 'front' 네임스페이스로 묶고 실행하는 그룹
    front_nav_group = GroupAction(actions=[
        PushRosNamespace(namespace),

        # 🚨 [유지] 전역 토픽 연결 (멀티봇 TF 및 Map 공유 핵심)
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        SetRemap(src='map', dst='/map'),

        # 0. EKF Sensor Fusion (Wheel Odom + IMU -> Final Odom TF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params_file, 
                {'use_sim_time': False}
            ],
            remappings=[('odometry/filtered', 'odom')]
        ),

        # 1. Controller
        Node(package='nav2_controller', 
             executable='controller_server', 
             output='screen', 
             parameters=[params_file, {'use_sim_time': False}], 
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        # 2. Planner
        Node(package='nav2_planner', 
             executable='planner_server', 
             output='screen', 
             parameters=[params_file, {'use_sim_time': False}]),
        # 3. Behaviors
        Node(package='nav2_behaviors', 
             executable='behavior_server', 
             output='screen',
               parameters=[params_file, {'use_sim_time': False}]),
        # 4. BT Navigator
        Node(package='nav2_bt_navigator', 
             executable='bt_navigator', 
             output='screen', 
             parameters=[params_file, {'use_sim_time': False}]),
        # 5. Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),

        # 6. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file, 
                {
                    'use_sim_time': False,
                    'yaml_filename': map_yaml_file # 동적 경로 주입
                }
            ]
        ),
        
        # 7. AMCL
        Node( 
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}]
        ),

        # 8. 통합 라이프사이클 매니저
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_front',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother'
                ]
            }]
        )
    ])

    # 9. RViz2 (네임스페이스 바깥에서 실행)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 10. Static TF (Footprint -> Base Link)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='f_base_footprint_to_f_base_link',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'front/base_footprint', 'front/base_link'],
        output='screen' 
    )

    # 🚨 [신규 추가] 11. Static TF (Base Link -> Laser Link)
    # AMCL이 라이다 스캔 데이터를 로봇 몸체에 맞추려면 반드시 필요한 뼈대입니다.
    # (높이(0.2m)나 전후 위치(0.1m) 등은 실제 센서 위치에 맞게 수정하세요)
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='f_base_link_to_f_laser',
        arguments=['0.1', '0.0', '0.2', '0.0', '0.0', '0.0', 'front/base_link', 'front/laser_link'],
        output='screen' 
    )

    ld.add_action(front_nav_group)
    ld.add_action(rviz_cmd)
    ld.add_action(tf_node)
    # ld.add_action(tf_laser_node) # 라이다 TF 추가

    return ld