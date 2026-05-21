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
    
    rviz_config_file = os.path.join(my_pkg_dir, 'rviz', 'front.rviz')
    map_yaml_file = os.path.join(my_pkg_dir, 'maps', '320.yaml')

    ld = LaunchDescription()

    # RViz2 그래픽 크래시 방지용 소프트웨어 렌더링 강제 설정
    set_gl_env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    ld.add_action(set_gl_env)

    # 🚨 [핵심] 모든 노드를 'front' 네임스페이스로 묶음
    front_nav_group = GroupAction(actions=[
        PushRosNamespace(namespace),

        # 전역 토픽 연결 (멀티봇 TF 및 Map 공유 핵심)
        SetRemap(src='tf', dst='/tf'),
        SetRemap(src='tf_static', dst='/tf_static'),
        SetRemap(src='map', dst='/map'),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[ekf_params_file, {'use_sim_time': False}],
            remappings=[('odometry/filtered', 'odom')]
        ),
        Node(package='nav2_controller', executable='controller_server', output='screen', 
             parameters=[params_file, {'use_sim_time': False}], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', output='screen', parameters=[params_file, {'use_sim_time': False}]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen', parameters=[params_file, {'use_sim_time': False}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', parameters=[params_file, {'use_sim_time': False}]),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother', output='screen',
            parameters=[params_file, {'use_sim_time': False}], remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[params_file, {'use_sim_time': False, 'yaml_filename': map_yaml_file}]
        ),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[params_file, {'use_sim_time': False}]),
        
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_front', output='screen',
            parameters=[{
                'use_sim_time': False, 'autostart': True,
                'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']
            }]
        )
    ])

    # RViz2 (네임스페이스 바깥에서 실행)
    rviz_cmd = Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[{'use_sim_time': False}], arguments=['-d', rviz_config_file], output='screen')

    # Static TF (Footprint -> Base Link)
    tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name='f_base_footprint_to_f_base_link',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'front/base_footprint', 'front/base_link'], output='screen' 
    )

    # 🚨 [오류 해결] 코스트맵 증발의 주원인! 라이다 TF 주석 해제 및 활성화!
    tf_laser_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name='f_base_link_to_f_laser',
        arguments=['0.1', '0.0', '0.2', '0.0', '0.0', '0.0', 'front/base_link', 'front/laser_link'], output='screen' 
    )

    ld.add_action(front_nav_group)
    ld.add_action(rviz_cmd)
    ld.add_action(tf_node)
    # ld.add_action(tf_laser_node) # 반드시 포함되어야 함

    return ld