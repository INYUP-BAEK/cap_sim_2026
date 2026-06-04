import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("cap_sim_2026")
    default_bt = os.path.join(
        pkg_dir,
        "bt_xml",
        "rear_cart_diff_nav_tree.xml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "back_distance_m",
                default_value="1.0",
                description="Goal distance behind the current rear robot pose.",
            ),
            DeclareLaunchArgument(
                "right_distance_m",
                default_value="1.0",
                description="Goal distance to the right of the current rear robot pose.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="",
                description=(
                    "Rear robot base frame. Empty uses base_footprint then base_link."
                ),
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Global frame for the NavigateToPose goal.",
            ),
            DeclareLaunchArgument(
                "action_name",
                default_value="navigate_to_pose",
                description="Rear robot NavigateToPose action name.",
            ),
            DeclareLaunchArgument(
                "behavior_tree",
                default_value=default_bt,
                description="Behavior tree XML for rear-cart differential mode.",
            ),
            DeclareLaunchArgument(
                "send_delay_sec",
                default_value="1.0",
                description="Delay before sampling TF and sending the test goal.",
            ),
            DeclareLaunchArgument(
                "publish_rear_cart_footprint",
                default_value="true",
                description="Publish rear-cart footprint while the test node runs.",
            ),
            DeclareLaunchArgument(
                "set_velocity_smoother",
                default_value="true",
                description="Set velocity_smoother to rear-cart differential limits.",
            ),
            Node(
                package="cap_sim_2026",
                executable="rear_cart_diff_test_goal.py",
                name="rear_cart_diff_test_goal",
                output="screen",
                parameters=[
                    {
                        "back_distance_m": ParameterValue(
                            LaunchConfiguration("back_distance_m"),
                            value_type=float,
                        ),
                        "right_distance_m": ParameterValue(
                            LaunchConfiguration("right_distance_m"),
                            value_type=float,
                        ),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "action_name": LaunchConfiguration("action_name"),
                        "behavior_tree": LaunchConfiguration("behavior_tree"),
                        "send_delay_sec": ParameterValue(
                            LaunchConfiguration("send_delay_sec"),
                            value_type=float,
                        ),
                        "publish_rear_cart_footprint": ParameterValue(
                            LaunchConfiguration("publish_rear_cart_footprint"),
                            value_type=bool,
                        ),
                        "set_velocity_smoother": ParameterValue(
                            LaunchConfiguration("set_velocity_smoother"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
