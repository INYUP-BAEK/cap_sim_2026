#!/usr/bin/env python3

import math
import os
import json
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    Point32,
    PointStamped,
    Polygon,
    Pose2D,
    PoseArray,
    PoseWithCovarianceStamped,
    PoseStamped,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String, UInt16

import tf2_geometry_msgs
import tf2_ros


class State(str, Enum):
    IDLE = "IDLE"
    PATROL = "PATROL"
    APPROACH_EXIT = "APPROACH_EXIT"
    WAIT_DETACH = "WAIT_DETACH"
    FRONT_CLEAR = "FRONT_CLEAR"
    REAR_ALIGN = "REAR_ALIGN"
    WAIT_PRECISE_POSE = "WAIT_PRECISE_POSE"
    DOCK_PREP = "DOCK_PREP"
    REAR_CART_PREP = "REAR_CART_PREP"
    WAIT_REAR_CART_ATTACH = "WAIT_REAR_CART_ATTACH"
    REAR_CART_GRIP_SETTLE = "REAR_CART_GRIP_SETTLE"
    REAR_CART_REJOIN = "REAR_CART_REJOIN"
    ROBOT_REJOIN = "ROBOT_REJOIN"
    WAIT_ATTACH = "WAIT_ATTACH"
    WAIT_FRONT_ATTACH = "WAIT_FRONT_ATTACH"
    WAIT_ROBOT_ATTACH = "WAIT_ROBOT_ATTACH"


class AutoNavCommander(Node):
    """Scenario runner that keeps runtime state sync independent from scenario flow."""

    def __init__(self):
        super().__init__("auto_nav_commander")

        self.declare_parameters(
            "",
            [
                ("initial_attached", True),
                ("enable_auto_scenario", True),
                ("allow_manual_goal_interrupt", False),
                ("scenario1_cart_station_x", -0.9610295295715332),
                ("scenario1_cart_station_y", 1.0462348461151123),
                ("scenario1_cart_count_after_docking", 3),
                ("scenario1_pickup_max_retries", 3),
                ("scenario2_cart_count_after_docking", 1),
                ("robot_rejoin_half_spacing", 0.5),
                ("footprint_publish_period_sec", 0.5),
                ("nav_server_timeout_sec", 2.0),
                ("route_goal_max_retries", 2),
                ("route_goal_retry_delay_sec", 1.0),
                ("route_goal_rejected_max_retries", 8),
                ("route_goal_rejected_retry_delay_sec", 2.5),
                ("rear_cart_prep_goal_max_retries", 20),
                ("rear_cart_prep_goal_retry_delay_sec", 3.0),
                ("recovery_release_settle_sec", 1.0),
                ("state_max_retries", 8),
                ("tf_retry_delay_sec", 0.5),
                ("route_arrival_tolerance", 0.35),
                ("route_tf_check_period_sec", 0.5),
                ("cart_exit_direct_route", False),
                ("front_clear_after_detach_delay_sec", 2.0),
                ("front_clear_distance", 0.5),
                ("scenario2_front_wait_clear_distance", 0.8),
                ("scenario2_front_wait_clear_timeout_sec", 0.0),
                ("scenario2_rear_cart_rejoin_back_distance", 1.0),
                ("rear_cart_rejoin_tf_xy_tolerance", 0.45),
                ("rear_cart_rejoin_tf_yaw_tolerance", 0.50),
                ("rear_cart_rejoin_tf_check_period_sec", 0.5),
                ("front_clear_speed", 0.12),
                ("front_clear_timeout_sec", 8.0),
                ("publish_front_initial_pose_before_clear", True),
                ("front_initial_pose_topic", "/front/initialpose"),
                ("front_initial_pose_source", "rear_geometry"),
                ("front_initial_pose_rear_offset_x", 0.48),
                ("front_initial_pose_rear_offset_y", 0.0),
                ("front_initial_pose_yaw_offset", 0.0),
                ("front_initial_pose_xy_covariance", 0.0004),
                ("front_initial_pose_yaw_covariance", 0.0009),
                ("detach_timeout_sec", 12.0),
                ("detach_release_max_retries", 1),
                ("detach_release_retry_delay_sec", 1.0),
                ("precise_pose_timeout_sec", 12.0),
                ("attach_timeout_sec", 180.0),
                ("scenario2_front_attach_timeout_sec", 240.0),
                ("scenario2_front_attach_max_retries", 1),
                ("scenario2_front_attach_retry_delay_sec", 2.0),
                ("rear_cart_grip_settle_delay_sec", 3.0),
                ("front_goal_timeout_sec", 30.0),
                ("dock_prep_front_goal_timeout_sec", 90.0),
                ("dock_goal_offset", 1.5),
                ("rear_dock_goal_offset", 1.5),
                ("front_dock_goal_offset", 1.0),
                ("rear_final_yaw_tolerance", 0.08),
                ("dock_prep_tf_xy_tolerance", 0.20),
                ("dock_prep_tf_yaw_tolerance", 0.35),
                ("dock_prep_start_xy_tolerance", 0.35),
                ("dock_prep_start_yaw_tolerance", 0.80),
                ("dock_prep_tf_check_period_sec", 0.5),
                ("precise_pose2d_frame", "base_link"),
                ("cart_marker_front_back_offset_m", 0.30),
                ("cart_marker_left_right_offset_m", 0.18),
                ("command_burst_duration_sec", 0.5),
                ("command_burst_period_sec", 0.1),
                ("auto_attach_after_dock_prep", False),
                ("front_nav_transport", "topic_proxy"),
                ("front_nav_goal_topic", "/front/scenario_nav_goal"),
                ("front_nav_cancel_topic", "/front/scenario_nav_cancel"),
                ("front_nav_result_topic", "/front/scenario_nav_result"),
                ("dock_prep_done_topic", "/dock_prep_done"),
                ("rl_docking_ready_topic", "/rl_docking_ready"),
                ("docking_target_topic", "/docking_target"),
                ("front_docking_target_topic", "/front/docking_target"),
                ("rl_docking_done_topic", "/rl_docking_done"),
                ("front_rl_docking_done_topic", "/front/rl_docking_done"),
                ("cart_count_topic", "/cart_count"),
                ("cart_count_after_docking", 1),
                ("manage_rear_nav2_lifecycle", True),
                # Temporarily keep rear Nav2 active after both dock-prep goals finish.
                ("pause_rear_nav2_on_dock_prep_done", False),
                ("resume_rear_nav2_on_scenario_start", True),
                (
                    "rear_nav2_lifecycle_service",
                    "/lifecycle_manager_navigation/manage_nodes",
                ),
                ("rear_nav2_lifecycle_service_timeout_sec", 2.0),
                ("rear_nav2_resume_delay_sec", 1.0),
                ("clear_rear_costmaps_on_cart_mode", True),
                (
                    "rear_global_costmap_clear_service",
                    "/global_costmap/clear_entirely_global_costmap",
                ),
                (
                    "rear_local_costmap_clear_service",
                    "/local_costmap/clear_entirely_local_costmap",
                ),
                ("rear_costmap_clear_service_timeout_sec", 0.5),
                ("rear_costmap_clear_settle_sec", 0.7),
                ("rear_costmap_clear_min_interval_sec", 1.0),
                ("cart_goal_cooldown_sec", 0.5),
                ("attached_cart_stop_distance", 1.5),
                ("detached_cart_stop_distance", 1.0),
            ],
        )

        self.state = State.IDLE
        self.is_attached = bool(self.get_parameter("initial_attached").value)
        self.cart_count = 0

        self.enable_auto_scenario = bool(
            self.get_parameter("enable_auto_scenario").value
        )
        self.allow_manual_goal_interrupt = bool(
            self.get_parameter("allow_manual_goal_interrupt").value
        )
        self.scenario1_cart_station_x = self.param_float(
            "scenario1_cart_station_x",
            0.0,
        )
        self.scenario1_cart_station_y = self.param_float(
            "scenario1_cart_station_y",
            0.0,
        )
        self.scenario1_cart_count_after_docking = max(
            1,
            self.param_int("scenario1_cart_count_after_docking", 3),
        )
        self.scenario1_pickup_max_retries = max(
            0,
            self.param_int("scenario1_pickup_max_retries", 3),
        )
        self.scenario2_cart_count_after_docking = max(
            1,
            self.param_int("scenario2_cart_count_after_docking", 1),
        )
        self.robot_rejoin_half_spacing = max(
            0.0,
            self.param_float("robot_rejoin_half_spacing", 0.5),
        )
        self.nav_server_timeout_sec = self.param_float("nav_server_timeout_sec", 2.0)
        self.route_goal_max_retries = self.param_int("route_goal_max_retries", 2)
        self.route_goal_retry_delay_sec = self.param_float(
            "route_goal_retry_delay_sec",
            1.0,
        )
        self.route_goal_rejected_max_retries = self.param_int(
            "route_goal_rejected_max_retries",
            8,
        )
        self.route_goal_rejected_retry_delay_sec = self.param_float(
            "route_goal_rejected_retry_delay_sec",
            2.5,
        )
        self.rear_cart_prep_goal_max_retries = max(
            0,
            self.param_int("rear_cart_prep_goal_max_retries", 20),
        )
        self.rear_cart_prep_goal_retry_delay_sec = max(
            0.1,
            self.param_float("rear_cart_prep_goal_retry_delay_sec", 3.0),
        )
        self.recovery_release_settle_sec = max(
            0.0,
            self.param_float("recovery_release_settle_sec", 1.0),
        )
        self.state_max_retries = self.param_int("state_max_retries", 8)
        self.tf_retry_delay_sec = self.param_float("tf_retry_delay_sec", 0.5)
        self.route_arrival_tolerance = self.param_float(
            "route_arrival_tolerance",
            0.35,
        )
        self.route_tf_check_period_sec = max(
            0.0,
            self.param_float("route_tf_check_period_sec", 0.5),
        )
        self.cart_exit_direct_route = bool(
            self.get_parameter("cart_exit_direct_route").value
        )
        self.front_clear_after_detach_delay_sec = self.param_float(
            "front_clear_after_detach_delay_sec",
            2.0,
        )
        self.front_clear_distance = self.param_float("front_clear_distance", 0.5)
        self.scenario2_front_wait_clear_distance = max(
            0.0,
            self.param_float("scenario2_front_wait_clear_distance", 0.5),
        )
        self.scenario2_front_wait_clear_timeout_sec = self.param_float(
            "scenario2_front_wait_clear_timeout_sec",
            0.0,
        )
        self.scenario2_rear_cart_rejoin_back_distance = max(
            0.0,
            self.param_float("scenario2_rear_cart_rejoin_back_distance", 1.0),
        )
        self.rear_cart_rejoin_tf_xy_tolerance = max(
            0.01,
            self.param_float("rear_cart_rejoin_tf_xy_tolerance", 0.45),
        )
        self.rear_cart_rejoin_tf_yaw_tolerance = max(
            0.01,
            self.param_float("rear_cart_rejoin_tf_yaw_tolerance", 0.80),
        )
        self.rear_cart_rejoin_tf_check_period_sec = max(
            0.0,
            self.param_float("rear_cart_rejoin_tf_check_period_sec", 0.5),
        )
        self.front_clear_speed = max(0.01, self.param_float("front_clear_speed", 0.12))
        self.front_clear_timeout_sec = self.param_float("front_clear_timeout_sec", 8.0)
        self.publish_front_initial_pose_before_clear = bool(
            self.get_parameter("publish_front_initial_pose_before_clear").value
        )
        self.front_initial_pose_topic = str(
            self.get_parameter("front_initial_pose_topic").value
        ).strip() or "/front/initialpose"
        self.front_initial_pose_source = str(
            self.get_parameter("front_initial_pose_source").value
        ).strip()
        self.front_initial_pose_rear_offset_x = float(
            self.get_parameter("front_initial_pose_rear_offset_x").value
        )
        self.front_initial_pose_rear_offset_y = float(
            self.get_parameter("front_initial_pose_rear_offset_y").value
        )
        self.front_initial_pose_yaw_offset = float(
            self.get_parameter("front_initial_pose_yaw_offset").value
        )
        self.front_initial_pose_xy_covariance = max(
            0.0,
            float(self.get_parameter("front_initial_pose_xy_covariance").value),
        )
        self.front_initial_pose_yaw_covariance = max(
            0.0,
            float(self.get_parameter("front_initial_pose_yaw_covariance").value),
        )
        self.detach_timeout_sec = self.param_float("detach_timeout_sec", 12.0)
        self.detach_release_max_retries = max(
            0,
            self.param_int("detach_release_max_retries", 1),
        )
        self.detach_release_retry_delay_sec = max(
            0.0,
            self.param_float("detach_release_retry_delay_sec", 1.0),
        )
        self.precise_pose_timeout_sec = self.param_float(
            "precise_pose_timeout_sec",
            12.0,
        )
        self.attach_timeout_sec = self.param_float("attach_timeout_sec", 180.0)
        self.scenario2_front_attach_timeout_sec = max(
            0.0,
            self.param_float("scenario2_front_attach_timeout_sec", 240.0),
        )
        self.scenario2_front_attach_max_retries = max(
            0,
            self.param_int("scenario2_front_attach_max_retries", 1),
        )
        self.scenario2_front_attach_retry_delay_sec = max(
            0.0,
            self.param_float("scenario2_front_attach_retry_delay_sec", 2.0),
        )
        self.rear_cart_grip_settle_delay_sec = max(
            0.0,
            self.param_float("rear_cart_grip_settle_delay_sec", 1.0),
        )
        self.front_goal_timeout_sec = self.param_float("front_goal_timeout_sec", 30.0)
        self.dock_prep_front_goal_timeout_sec = self.param_float(
            "dock_prep_front_goal_timeout_sec",
            90.0,
        )
        self.dock_goal_offset = self.param_float("dock_goal_offset", 2.0)
        self.rear_dock_goal_offset = max(
            0.0,
            self.param_float("rear_dock_goal_offset", 1.5),
        )
        self.front_dock_goal_offset = max(
            0.0,
            self.param_float("front_dock_goal_offset", 1.0),
        )
        self.rear_final_yaw_tolerance = max(
            0.01,
            self.param_float("rear_final_yaw_tolerance", 0.08),
        )
        self.dock_prep_tf_xy_tolerance = max(
            0.01,
            self.param_float("dock_prep_tf_xy_tolerance", 0.20),
        )
        self.dock_prep_tf_yaw_tolerance = max(
            0.01,
            self.param_float("dock_prep_tf_yaw_tolerance", 0.35),
        )
        self.dock_prep_start_xy_tolerance = max(
            self.dock_prep_tf_xy_tolerance,
            self.param_float("dock_prep_start_xy_tolerance", 0.35),
        )
        self.dock_prep_start_yaw_tolerance = max(
            self.dock_prep_tf_yaw_tolerance,
            self.param_float("dock_prep_start_yaw_tolerance", 0.80),
        )
        self.dock_prep_tf_check_period_sec = max(
            0.0,
            self.param_float("dock_prep_tf_check_period_sec", 0.5),
        )
        self.precise_pose2d_frame = self.normalize_frame_id(
            self.get_parameter("precise_pose2d_frame").value
        )
        self.cart_marker_front_back_offset_m = max(
            0.0,
            self.param_float("cart_marker_front_back_offset_m", 0.30),
        )
        self.cart_marker_left_right_offset_m = max(
            0.0,
            self.param_float("cart_marker_left_right_offset_m", 0.18),
        )
        self.command_burst_duration_sec = self.param_float(
            "command_burst_duration_sec",
            0.5,
        )
        self.command_burst_period_sec = max(
            0.02,
            self.param_float("command_burst_period_sec", 0.1),
        )
        self.auto_attach_after_dock_prep = bool(
            self.get_parameter("auto_attach_after_dock_prep").value
        )
        self.front_nav_transport = str(
            self.get_parameter("front_nav_transport").value
        ).strip()
        self.front_nav_goal_topic = str(
            self.get_parameter("front_nav_goal_topic").value
        ).strip()
        self.front_nav_cancel_topic = str(
            self.get_parameter("front_nav_cancel_topic").value
        ).strip()
        self.front_nav_result_topic = str(
            self.get_parameter("front_nav_result_topic").value
        ).strip()
        self.dock_prep_done_topic = str(
            self.get_parameter("dock_prep_done_topic").value
        ).strip()
        self.rl_docking_ready_topic = str(
            self.get_parameter("rl_docking_ready_topic").value
        ).strip()
        self.docking_target_topic = str(
            self.get_parameter("docking_target_topic").value
        ).strip()
        self.front_docking_target_topic = str(
            self.get_parameter("front_docking_target_topic").value
        ).strip()
        self.rl_docking_done_topic = str(
            self.get_parameter("rl_docking_done_topic").value
        ).strip()
        self.front_rl_docking_done_topic = str(
            self.get_parameter("front_rl_docking_done_topic").value
        ).strip()
        self.cart_count_topic = str(
            self.get_parameter("cart_count_topic").value
        ).strip() or "/cart_count"
        self.cart_count_after_docking = max(
            1,
            self.param_int("cart_count_after_docking", 1),
        )
        self.manage_rear_nav2_lifecycle = bool(
            self.get_parameter("manage_rear_nav2_lifecycle").value
        )
        self.pause_rear_nav2_on_dock_prep_done = bool(
            self.get_parameter("pause_rear_nav2_on_dock_prep_done").value
        )
        self.resume_rear_nav2_on_scenario_start = bool(
            self.get_parameter("resume_rear_nav2_on_scenario_start").value
        )
        self.rear_nav2_lifecycle_service = str(
            self.get_parameter("rear_nav2_lifecycle_service").value
        ).strip()
        self.rear_nav2_lifecycle_service_timeout_sec = self.param_float(
            "rear_nav2_lifecycle_service_timeout_sec",
            2.0,
        )
        self.rear_nav2_resume_delay_sec = self.param_float(
            "rear_nav2_resume_delay_sec",
            1.0,
        )
        self.clear_rear_costmaps_on_cart_mode = bool(
            self.get_parameter("clear_rear_costmaps_on_cart_mode").value
        )
        self.rear_global_costmap_clear_service = str(
            self.get_parameter("rear_global_costmap_clear_service").value
        ).strip()
        self.rear_local_costmap_clear_service = str(
            self.get_parameter("rear_local_costmap_clear_service").value
        ).strip()
        self.rear_costmap_clear_service_timeout_sec = self.param_float(
            "rear_costmap_clear_service_timeout_sec",
            0.5,
        )
        self.rear_costmap_clear_settle_sec = self.param_float(
            "rear_costmap_clear_settle_sec",
            0.7,
        )
        self.rear_costmap_clear_min_interval_sec = self.param_float(
            "rear_costmap_clear_min_interval_sec",
            1.0,
        )

        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.rear_cart_diff_bt_path = os.path.join(
            pkg_dir,
            "bt_xml",
            "rear_cart_diff_nav_tree.xml",
        )
        self.ackermann_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_nav_tree.xml")
        self.ackermann_cart_bt_path = os.path.join(
            pkg_dir,
            "bt_xml",
            "ackermann_cart2_nav_tree.xml",
        )

        self.rear_nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.front_nav_client = ActionClient(
            self,
            NavigateToPose,
            "/front/navigate_to_pose",
        )
        self.rear_nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            self.rear_nav2_lifecycle_service,
        )
        self.rear_global_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            self.rear_global_costmap_clear_service,
        )
        self.rear_local_costmap_clear_client = self.create_client(
            ClearEntireCostmap,
            self.rear_local_costmap_clear_service,
        )

        self.active_rear_goal_handle = None
        self.active_rear_goal_seq = 0
        self.active_rear_goal_pending = False
        self.active_rear_goal_label = None

        self.active_front_goal_handle = None
        self.active_front_goal_seq = 0
        self.active_front_goal_pending = False
        self.active_front_goal_label = None
        self.active_front_proxy_goal_id = 0

        # Waypoints accept (x, y), (x, y, yaw_rad), (x, y, qz, qw),
        # (x, y, qx, qy, qz, qw), or PoseStamped-like dicts.
        # self.patrol_path = [(6.15, -0.52), (7.02, 1.95)]
        # self.patrol_path = [
        #     (7.563, -16.764, 0.712, 0.702),
        #     (7.099, -4.331, 0.704, 0.711),
        #     (-3.604, -0.417, -1.000, 0.009),
        #     (-8.901, -4.697, -0.712, 0.702),
        #     (-9.068, -16.308, -0.681, 0.732),
        #     (-1.477, -20.036, 0.008, 1.000),
        #     (7.567, -16.613, 0.719, 0.695),
        # ]
        self.patrol_path = [
            (4.3804, -0.2327, -0.9999, 0.0095),
            (-6.005, -0.3820, -0.9999, 0.0095),
            (-8.682, -5.2208, -0.738, 0.6747),
        ]
        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0
        self.active_route_patrol_start_index = 0
        self.route_goal_retry_count = 0
        self.current_patrol_waypoint_index = 0
        self.pending_patrol_resume_after_docking = False
        self.resume_patrol_waypoint_index = 0
        self.rear_nav2_paused = False
        self.last_rear_costmap_clear_time = None
        self.active_scenario_id = 0
        self.detach_pose = None
        self.rear_cart_attached = False
        self.scenario_recovery_active = False
        self.scenario1_pickup_retry_count = 0
        self.rear_cart_prep_goal_retry_count = 0
        self.rejoin_mode = None
        self.rejoin_rear_goal_done = False
        self.rejoin_front_goal_done = False

        self.detected_cart_pose = None
        self.pending_precise_pose_msg = None
        self.pending_precise_pose_type = None
        self.last_dock_prep_rear_goal = None
        self.last_dock_prep_front_goal = None
        self.last_rear_cart_rejoin_goal = None
        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.last_cart_goal_time = None
        self.front_clear_tf_retries = 0
        self.front_clear_goal_retry_count = 0
        self.front_wait_clear_goal_retry_count = 0
        self.rear_align_tf_retries = 0
        self.rear_align_goal_retry_count = 0
        self.dock_prep_rear_goal_retry_count = 0
        self.rear_cart_prep_goal_retry_count = 0
        self.precise_goal_retries = 0
        self.detach_release_retry_count = 0
        self.scenario2_front_attach_retry_count = 0
        self.front_rl_docking_started = False
        self.front_rl_docking_done = False
        self.rear_rl_docking_done = False
        self.rear_rl_docking_started = False
        self.docking_state_confirmed = bool(self.is_attached)
        self.scenario_timers = {}

        self.global_footprint_pub = self.create_publisher(
            Polygon,
            "/global_costmap/footprint",
            10,
        )
        self.local_footprint_pub = self.create_publisher(
            Polygon,
            "/local_costmap/footprint",
            10,
        )
        self.front_global_footprint_pub = self.create_publisher(
            Polygon,
            "/front/global_costmap/footprint",
            10,
        )
        self.front_local_footprint_pub = self.create_publisher(
            Polygon,
            "/front/local_costmap/footprint",
            10,
        )
        self.gripper_toggle_pub = self.create_publisher(Bool, "/gripper_toggle", 10)
        self.front_home_pub = self.create_publisher(Bool, "/front/home", 10)
        self.front_robot_docking_pub = self.create_publisher(
            Bool,
            "/front/robot_docking",
            10,
        )
        self.front_nav_goal_pub = self.create_publisher(
            String,
            self.front_nav_goal_topic,
            10,
        )
        self.front_nav_cancel_pub = self.create_publisher(
            String,
            self.front_nav_cancel_topic,
            10,
        )
        self.dock_prep_done_pub = self.create_publisher(
            Bool,
            self.dock_prep_done_topic,
            10,
        )
        self.rl_docking_ready_pub = self.create_publisher(
            Bool,
            self.rl_docking_ready_topic,
            10,
        )
        self.docking_target_pub = self.create_publisher(
            Int32,
            self.docking_target_topic,
            10,
        )
        self.front_docking_target_pub = self.create_publisher(
            Int32,
            self.front_docking_target_topic,
            10,
        )
        self.cart_count_pub = self.create_publisher(
            UInt16,
            self.cart_count_topic,
            10,
        )
        self.front_initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.front_initial_pose_topic,
            10,
        )
        self.rear_joy_sig_pub = self.create_publisher(Bool, "/joy_control_sig", 10)
        self.front_joy_sig_pub = self.create_publisher(
            Bool,
            "/front/joy_control_sig",
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        tf_static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        cart_target_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            qos=tf_qos,
            static_qos=tf_static_qos,
        )

        self.topic_subs = [
            self.create_subscription(Bool, "/docking_state", self.docking_callback, 10),
            self.create_subscription(
                Bool,
                self.rl_docking_done_topic,
                self.rl_docking_done_callback,
                10,
            ),
            self.create_subscription(
                Bool,
                self.front_rl_docking_done_topic,
                self.front_rl_docking_done_callback,
                10,
            ),
            self.create_subscription(
                UInt16,
                self.cart_count_topic,
                self.cart_count_callback,
                10,
            ),
            self.create_subscription(
                PoseStamped,
                "/mission_goal",
                self.mission_goal_callback,
                10,
            ),
            self.create_subscription(
                PointStamped,
                "/zed_yolo/global_cart_target",
                self.cart_target_callback,
                cart_target_qos,
            ),
            self.create_subscription(
                PoseStamped,
                "/vision/cart_precise_pose",
                self.precise_pose_callback,
                10,
            ),
            self.create_subscription(
                Pose2D,
                "/vision/cart_precise_pose_2d",
                self.precise_pose2d_callback,
                10,
            ),
            self.create_subscription(
                PointStamped,
                "/rear/target_pose",
                self.precise_target_point_callback,
                10,
            ),
            self.create_subscription(
                String,
                self.front_nav_result_topic,
                self.front_nav_proxy_result_callback,
                10,
            ),
            self.create_subscription(
                Int32,
                "/start_patrol_mission",
                self.start_mission_callback,
                10,
            ),
            self.create_subscription(
                Path, "/mission_path", self.mission_path_callback, 10
            ),
            self.create_subscription(
                PoseArray,
                "/mission_waypoints",
                self.mission_waypoints_callback,
                10,
            ),
        ]

        period = max(0.1, self.param_float("footprint_publish_period_sec", 0.5))
        self.footprint_timer = self.create_timer(period, self.publish_footprints)

        self.publish_dock_prep_done(False)
        self.publish_rl_docking_ready(False)
        self.update_dynamic_state("initial")
        self.get_logger().info(
            "Scenario runner ready. state sync is always active, auto_scenario=%s"
            % ("true" if self.enable_auto_scenario else "false")
        )

    # ------------------------------------------------------------------
    # Always-on state sync
    # ------------------------------------------------------------------
    def docking_callback(self, msg: Bool):
        next_attached = bool(msg.data)
        if self.should_ignore_docking_state_for_rear_cart(next_attached):
            if self.is_attached:
                self.is_attached = False
                self.get_logger().warn(
                    "forcing is_attached=false because scenario 2 rear-cart "
                    "pull-out is a detached differential state."
                )
                self.update_dynamic_state("rear_cart_docking_state_filter")
            else:
                self.publish_footprints()
            self.get_logger().warn(
                f"docking_state=true ignored during {self.state.value}; "
                "rear-cart pull-out keeps /docking_state=false until front attach."
            )
            return

        if self.is_attached != next_attached:
            self.is_attached = next_attached
            self.get_logger().info(f"docking_state -> attached={self.is_attached}")
            self.update_dynamic_state("docking_state")
        else:
            self.publish_footprints()

        self.handle_docking_transition(next_attached)

    def should_ignore_docking_state_for_rear_cart(self, attached: bool):
        if not attached:
            return False
        if self.active_scenario_id != 2:
            return False
        return self.state in (
            State.REAR_CART_PREP,
            State.WAIT_REAR_CART_ATTACH,
            State.REAR_CART_GRIP_SETTLE,
            State.REAR_CART_REJOIN,
        )

    def cart_count_callback(self, msg: UInt16):
        if self.cart_count == msg.data:
            self.publish_footprints()
            return

        self.set_cart_count(int(msg.data), "cart_count", publish=False)

    def rl_docking_done_callback(self, msg: Bool):
        if not bool(msg.data):
            return

        if self.state == State.WAIT_REAR_CART_ATTACH:
            if not self.rear_rl_docking_started:
                self.get_logger().warn(
                    "rear cart docking done ignored because rear RL has not started."
                )
                return
            if self.rear_rl_docking_done:
                return
            self.rear_rl_docking_done = True
            self.get_logger().info("rear cart rl_docking_done=true")
            self.set_rear_cart_attached(True, "rear_cart_attached")
            self.cancel_timer("attach_timeout")
            delay = self.rear_cart_grip_settle_delay_sec
            self.set_state(State.REAR_CART_GRIP_SETTLE, "rear_cart_rl_done")
            self.get_logger().info(
                "rear cart grip settle wait before rejoin: %.2fs" % delay
            )
            self.schedule_once(
                "rear_cart_grip_settle",
                delay,
                self.start_rear_cart_rejoin,
            )
            return

        if self.state == State.WAIT_ROBOT_ATTACH:
            if not self.rear_rl_docking_started:
                self.get_logger().warn(
                    "rear robot docking done ignored because rear RL has not started."
                )
                return
            if not self.rear_rl_docking_done:
                self.rear_rl_docking_done = True
                self.get_logger().info("rear robot rl_docking_done=true")
            self.check_robot_attach_done("rear_rl_docking_done")
            return

        if self.state == State.WAIT_FRONT_ATTACH:
            if not self.rear_rl_docking_done:
                self.rear_rl_docking_done = True
                self.get_logger().info("rear cart rl_docking_done=true")
            return

        if self.state != State.WAIT_ATTACH:
            self.get_logger().warn(
                f"rear rl docking done ignored because scenario state={self.state.value}"
            )
            return

        if not self.rear_rl_docking_started:
            self.get_logger().warn(
                "rear rl docking done ignored because rear RL has not started."
            )
            return

        if not self.rear_rl_docking_done:
            self.rear_rl_docking_done = True
            self.get_logger().info("rear rl_docking_done=true")
        self.check_rl_docking_sequence_done("rear_rl_docking_done")

    def front_rl_docking_done_callback(self, msg: Bool):
        if not bool(msg.data):
            return

        if self.state == State.WAIT_FRONT_ATTACH:
            if not self.front_rl_docking_started:
                self.get_logger().warn(
                    "front cart docking done ignored because front RL has not started."
                )
                return

            if not self.front_rl_docking_done:
                self.front_rl_docking_done = True
                self.get_logger().info("front cart rl_docking_done=true")
            self.check_front_attach_done("front_rl_docking_done")
            return

        if self.state == State.WAIT_ROBOT_ATTACH:
            if not self.front_rl_docking_started:
                self.get_logger().warn(
                    "front robot docking done ignored because front RL has not started."
                )
                return

            if not self.front_rl_docking_done:
                self.front_rl_docking_done = True
                self.get_logger().info("front robot rl_docking_done=true")
            self.check_robot_attach_done("front_rl_docking_done")
            return

        if self.state != State.WAIT_ATTACH:
            self.get_logger().warn(
                f"front rl docking done ignored because scenario state={self.state.value}"
            )
            return

        if not self.front_rl_docking_started:
            self.get_logger().warn(
                "front rl docking done ignored because front RL has not started."
            )
            return

        if not self.front_rl_docking_done:
            self.front_rl_docking_done = True
            self.get_logger().info("front rl_docking_done=true")

        if not self.rear_rl_docking_started:
            self.rear_rl_docking_started = True
            self.publish_int_burst(
                "scenario1_rear_attach_start",
                self.docking_target_pub,
                2,
            )
            self.get_logger().info(
                "front RL docking complete. Rear docking target start command published."
            )

        self.check_rl_docking_sequence_done("front_rl_docking_done")

    def set_cart_count(self, count: int, reason: str, publish: bool):
        next_count = max(0, int(count))
        changed = self.cart_count != next_count
        self.cart_count = next_count
        if self.cart_count <= 0:
            self.last_rear_costmap_clear_time = None

        if publish:
            self.publish_cart_count(self.cart_count)

        if changed:
            self.get_logger().info(f"cart_count -> {self.cart_count} ({reason})")
            self.update_dynamic_state(reason)
        else:
            self.publish_footprints()

    def set_rear_cart_attached(self, attached: bool, reason: str):
        next_attached = bool(attached)
        forced_detached = False
        if next_attached and self.is_attached:
            self.is_attached = False
            forced_detached = True
            self.get_logger().warn(
                "rear_cart_attached=true requires is_attached=false; "
                "forcing detached differential state."
            )

        changed = self.rear_cart_attached != next_attached
        self.rear_cart_attached = next_attached
        if changed:
            self.get_logger().info(
                f"rear_cart_attached -> {self.rear_cart_attached} ({reason})"
            )
        if changed or forced_detached:
            self.update_dynamic_state(reason)
        else:
            self.publish_footprints()

    def scenario_cart_count_after_docking(self):
        if self.active_scenario_id == 1:
            return self.scenario1_cart_count_after_docking
        if self.active_scenario_id == 2:
            return self.scenario2_cart_count_after_docking
        return self.cart_count_after_docking

    def update_dynamic_state(self, reason: str):
        self.publish_footprints()
        smoother_params, mode = self.current_velocity_params()
        self.send_parameters_to_node("/velocity_smoother", smoother_params)
        self.get_logger().info(f"dynamic update ({reason}): {mode}")

    def current_velocity_params(self):
        if self.rear_cart_attached:
            return (
                {
                    "max_velocity": [0.0, 0.0, 0.35],
                    "min_velocity": [-0.12, 0.0, -0.35],
                    "max_accel": [0.20, 0.0, 1.0],
                    "max_decel": [-0.20, 0.0, -1.0],
                },
                "differential rear-cart mode",
            )

        if self.is_attached and self.cart_count >= 1:
            return (
                {
                    "max_velocity": [0.18, 0.0, 0.55],
                    "min_velocity": [-0.1, 0.0, -0.55],
                    "max_accel": [0.12, 0.0, 0.25],
                    "max_decel": [-0.15, 0.0, -0.3],
                },
                f"ackermann cart mode ({self.cart_count} cart)",
            )

        if self.is_attached:
            return (
                {
                    "max_velocity": [0.25, 0.0, 1.2],
                    "min_velocity": [-0.15, 0.0, -1.2],
                    "max_accel": [0.3, 0.0, 1.2],
                    "max_decel": [-0.5, 0.0, -1.2],
                },
                "ackermann direct mode",
            )

        return (
            {
                "max_velocity": [0.35, 0.0, 1.2],
                "min_velocity": [-0.35, 0.0, -1.2],
                "max_accel": [0.5, 0.0, 6.0],
                "max_decel": [-0.5, 0.0, -6.0],
            },
            "differential detached mode",
        )

    def publish_footprints(self):
        rear_msg, front_msg = self.current_footprints()
        self.global_footprint_pub.publish(rear_msg)
        self.local_footprint_pub.publish(rear_msg)
        self.front_global_footprint_pub.publish(front_msg)
        self.front_local_footprint_pub.publish(front_msg)

    def current_footprints(self):
        rear_width = 0.25
        rear_bumper_x = -0.3

        if self.rear_cart_attached:
            rear_front_x = self.wheelbase_from_cart_count(1) + 0.3
            return (
                self.create_polygon(rear_front_x, rear_bumper_x, rear_width),
                self.create_polygon(0.3, -0.3, 0.25),
            )

        if self.is_attached:
            rear_front_x = self.wheelbase_from_cart_count(self.cart_count) + 0.3
            return (
                self.create_polygon(rear_front_x, rear_bumper_x, rear_width),
                self.create_polygon(0.01, -0.01, 0.01),
            )

        return (
            self.create_polygon(0.3, rear_bumper_x, rear_width),
            self.create_polygon(0.3, -0.3, 0.25),
        )

    def wheelbase_from_cart_count(self, cart_count: int):
        if cart_count <= 0:
            return 0.48
        return 1.30 + 0.15 * float(cart_count - 1)

    # ------------------------------------------------------------------
    # Mission entry points
    # ------------------------------------------------------------------
    def start_mission_callback(self, msg: Int32):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return

        scenario_id = int(msg.data)
        if scenario_id not in (1, 2):
            self.get_logger().warn(
                f"start_patrol_mission ignored: scenario_id must be 1 or 2, got={scenario_id}"
            )
            return

        self.start_patrol_route(
            self.patrol_path,
            f"start_patrol_mission:{scenario_id}",
            scenario_id=scenario_id,
        )

    def mission_path_callback(self, msg: Path):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return

        frame_id = self.normalize_frame_id(msg.header.frame_id) or "map"
        points = []
        for index, pose_stamped in enumerate(msg.poses):
            source_frame = self.normalize_frame_id(pose_stamped.header.frame_id) or frame_id
            point = self.pose_to_map_waypoint(
                pose_stamped.pose,
                source_frame,
                f"path[{index}]",
            )
            if point is not None:
                points.append(point)

        self.update_patrol_path(points, "mission_path")

    def mission_waypoints_callback(self, msg: PoseArray):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return

        source_frame = self.normalize_frame_id(msg.header.frame_id) or "map"
        points = []
        for index, pose in enumerate(msg.poses):
            point = self.pose_to_map_waypoint(
                pose,
                source_frame,
                f"waypoint[{index}]",
            )
            if point is not None:
                points.append(point)

        self.update_patrol_path(points, "mission_waypoints")

    def mission_goal_callback(self, msg: PoseStamped):
        if self.state != State.IDLE:
            if not self.allow_manual_goal_interrupt:
                self.get_logger().warn(
                    f"mission_goal ignored because scenario state={self.state.value}"
                )
                return
            self.abort_scenario("manual mission_goal interrupt", return_joystick=False)

        if not self.pose_xy_is_finite(msg):
            self.get_logger().warn("mission_goal has invalid x/y. Ignored.")
            return
        if not msg.header.frame_id:
            msg.header.frame_id = "map"

        wait_for_resume = self.resume_rear_nav2_on_scenario_start and self.rear_nav2_paused
        if self.resume_rear_nav2_on_scenario_start:
            self.resume_rear_nav2("manual_goal")
        self.enable_navigation_control()
        self.update_dynamic_state("manual_goal")
        if wait_for_resume:
            self.schedule_once(
                "rear_nav2_resume_manual_goal",
                self.rear_nav2_resume_delay_sec,
                lambda goal=msg: self.send_rear_goal(
                    goal,
                    "MANUAL",
                    self.current_behavior_tree(),
                ),
            )
            return
        self.send_rear_goal(msg, "MANUAL", self.current_behavior_tree())

    def update_patrol_path(self, points, source: str):
        if self.state != State.IDLE:
            self.get_logger().warn(
                f"{source} ignored because scenario state={self.state.value}"
            )
            return False

        clean_points = self.clean_points(points)
        if len(clean_points) < 2:
            self.get_logger().error(
                f"{source} needs at least 2 valid waypoints. got={len(clean_points)}"
            )
            return False

        self.patrol_path = clean_points
        self.current_patrol_waypoint_index = 0
        self.get_logger().info(
            f"{source} updated patrol path: {len(clean_points)} waypoints. "
            "Publish /start_patrol_mission Int32(data=1 or 2) to start."
        )
        return True

    def start_patrol_route(self, points, source: str, scenario_id=2):
        if self.state != State.IDLE:
            self.get_logger().warn(
                f"{source} ignored because scenario state={self.state.value}"
            )
            return False

        clean_points = self.clean_points(points)
        if len(clean_points) < 2:
            self.get_logger().error(
                f"{source} needs at least 2 valid waypoints. got={len(clean_points)}"
            )
            return False

        if not self.is_attached:
            self.get_logger().warn(
                "patrol is starting while docking_state=false. "
                "State sync stays live; check the physical attach state if this is unexpected."
            )

        self.cancel_all_timers()
        self.clear_route()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_precise_pose()
        self.detected_cart_pose = None
        self.patrol_path = clean_points
        self.current_patrol_waypoint_index = 0
        self.clear_patrol_resume_after_docking()
        self.active_scenario_id = int(scenario_id)
        self.detach_pose = None
        self.set_rear_cart_attached(False, f"{source}_start")
        self.scenario_recovery_active = False
        self.scenario1_pickup_retry_count = 0
        self.detach_release_retry_count = 0
        self.scenario2_front_attach_retry_count = 0
        self.rejoin_mode = None
        self.rejoin_rear_goal_done = False
        self.rejoin_front_goal_done = False
        self.set_cart_count(0, f"{source}_start", publish=True)

        self.enable_navigation_control()
        wait_for_resume = self.resume_rear_nav2_on_scenario_start and self.rear_nav2_paused
        if self.resume_rear_nav2_on_scenario_start:
            self.resume_rear_nav2(source)
        if self.active_scenario_id == 1:
            self.set_state(State.APPROACH_EXIT, source)
            start_callback = self.start_scenario1_pickup_route
        else:
            self.set_state(State.PATROL, source)
            start_callback = lambda route=clean_points: self.start_route(route, "PATROL")

        if wait_for_resume:
            self.schedule_once(
                "rear_nav2_resume_start_route",
                self.rear_nav2_resume_delay_sec,
                lambda callback=start_callback: callback()
                if self.state in (State.PATROL, State.APPROACH_EXIT)
                else None,
            )
            return True
        return start_callback()

    def start_scenario1_pickup_route(self):
        if self.state != State.APPROACH_EXIT:
            return False

        station_x = float(self.scenario1_cart_station_x)
        station_y = float(self.scenario1_cart_station_y)
        self.detected_cart_pose = self.create_pose_stamped(station_x, station_y, 0.0)

        current_progress = self.current_progress_on_path()
        exit_projection = self.closest_point_on_path(
            station_x,
            station_y,
            min_progress=current_progress + 0.05,
        )
        exit_route = self.build_route_to_exit(exit_projection, current_progress)

        self.get_logger().info(
            "scenario 1 cart station=(%.2f, %.2f). exit=(%.2f, %.2f), "
            "exit_route_points=%d"
            % (
                station_x,
                station_y,
                exit_projection["point"][0],
                exit_projection["point"][1],
                len(exit_route),
            )
        )
        return self.start_route(
            exit_route,
            "EXIT",
            final_yaw=exit_projection["path_yaw"],
        )

    # ------------------------------------------------------------------
    # Cart detection and route-to-exit
    # ------------------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        if self.state != State.PATROL:
            return
        if self.active_scenario_id != 2:
            return
        if self.cart_count >= 1:
            return
        if not msg.header.frame_id:
            self.get_logger().warn("cart target frame_id is empty. Ignored.")
            return

        now = self.get_clock().now()
        cooldown = self.param_float("cart_goal_cooldown_sec", 0.5)
        if self.last_cart_goal_time is not None:
            elapsed = (now - self.last_cart_goal_time).nanoseconds * 1e-9
            if elapsed < cooldown:
                return

        cart_pose = self.point_stamped_to_map_pose(msg)
        if cart_pose is None:
            return

        self.detected_cart_pose = cart_pose
        self.last_cart_goal_time = now

        robot_xy = self.get_robot_xy_in_map()
        if robot_xy is not None:
            distance_to_cart = math.hypot(
                cart_pose.pose.position.x - robot_xy[0],
                cart_pose.pose.position.y - robot_xy[1],
            )
        else:
            distance_to_cart = math.hypot(msg.point.x, msg.point.y)

        if distance_to_cart <= self.cart_stop_distance():
            self.get_logger().info(
                "cart target is close enough; starting detach sequence from current area."
            )
            self.prepare_patrol_resume_after_docking(
                "cart close",
                use_active_route=True,
            )
            self.clear_route()
            self.cancel_rear_goal()
            self.start_detach_sequence()
            return

        current_progress = self.current_progress_on_path()
        exit_projection = self.closest_point_on_path(
            cart_pose.pose.position.x,
            cart_pose.pose.position.y,
            min_progress=current_progress + 0.05,
        )
        exit_route = self.build_route_to_exit(exit_projection, current_progress)

        self.get_logger().info(
            "cart detected at map=(%.2f, %.2f). exit=(%.2f, %.2f), exit_route_points=%d"
            % (
                cart_pose.pose.position.x,
                cart_pose.pose.position.y,
                exit_projection["point"][0],
                exit_projection["point"][1],
                len(exit_route),
            )
        )

        self.cancel_rear_goal()
        self.prepare_patrol_resume_after_docking(
            "cart_detected",
            use_active_route=True,
        )
        self.clear_route()
        self.set_state(State.APPROACH_EXIT, "cart_detected")
        self.start_route(exit_route, "EXIT", final_yaw=exit_projection["path_yaw"])

    # ------------------------------------------------------------------
    # Scenario stages
    # ------------------------------------------------------------------
    def start_detach_sequence(self):
        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.record_detach_pose()
        self.front_clear_tf_retries = 0
        self.front_clear_goal_retry_count = 0
        self.front_wait_clear_goal_retry_count = 0
        self.rear_align_tf_retries = 0
        self.rear_align_goal_retry_count = 0
        self.dock_prep_rear_goal_retry_count = 0
        self.rear_cart_prep_goal_retry_count = 0
        self.precise_goal_retries = 0
        self.detach_release_retry_count = 0

        if not self.is_attached:
            self.get_logger().info("already detached; skipping detach wait.")
            self.schedule_front_clear()
            return

        self.set_state(State.WAIT_DETACH, "detach_request")
        self.enable_navigation_control()
        self.publish_bool_burst("gripper_release", self.gripper_toggle_pub, False)
        self.publish_bool_burst("front_home", self.front_home_pub, True)
        self.publish_docking_target_burst(0, "detach_docking_target_reset")

        self.schedule_detach_timeout()

        self.get_logger().info(
            "detach requested. Waiting for /docking_state=false from controller or joystick."
        )

    def schedule_detach_timeout(self):
        if self.detach_timeout_sec <= 0.0:
            return
        self.schedule_once(
            "detach_timeout",
            self.detach_timeout_sec,
            self.handle_detach_timeout,
        )

    def retry_detach_release(self, reason: str):
        if self.state != State.WAIT_DETACH:
            return False
        if not self.is_attached:
            self.cancel_timer("detach_timeout")
            self.schedule_front_clear()
            return True
        if self.detach_release_retry_count >= self.detach_release_max_retries:
            return False

        self.detach_release_retry_count += 1
        delay = self.detach_release_retry_delay_sec
        self.cancel_timer("detach_timeout")
        self.enable_navigation_control()
        self.publish_docking_target_burst(0, "detach_retry_docking_target_reset")
        self.publish_bool_burst("gripper_release", self.gripper_toggle_pub, False)
        self.publish_bool_burst("front_home", self.front_home_pub, True)
        self.get_logger().warn(
            "detach release timeout: %s. retry %d/%d after %.1fs"
            % (
                reason,
                self.detach_release_retry_count,
                self.detach_release_max_retries,
                delay,
            )
        )

        self.schedule_once(
            "detach_release_retry_wait",
            delay,
            lambda: self.schedule_detach_timeout()
            if self.state == State.WAIT_DETACH
            else None,
        )
        return True

    def record_detach_pose(self):
        pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if pose is None:
            self.get_logger().warn(
                "detach pose unavailable; fallback/rejoin goals may be unavailable."
            )
            return

        self.detach_pose = pose
        self.get_logger().info(
            "detach pose recorded: x=%.2f, y=%.2f, yaw=%.2f"
            % (pose[0], pose[1], pose[2])
        )

    def handle_docking_transition(self, attached: bool):
        if self.state == State.WAIT_DETACH and not attached:
            self.cancel_timer("detach_timeout")
            self.cancel_timer("detach_release_retry_wait")
            self.detach_release_retry_count = 0
            self.schedule_front_clear()
            return

        if self.state in (
            State.WAIT_ATTACH,
            State.WAIT_FRONT_ATTACH,
            State.WAIT_ROBOT_ATTACH,
        ):
            if attached:
                if not self.docking_state_confirmed:
                    self.get_logger().info("docking_state=true confirmed")
                self.docking_state_confirmed = True
            else:
                self.docking_state_confirmed = False

            if self.state == State.WAIT_ATTACH:
                self.check_rl_docking_sequence_done("docking_state")
            elif self.state == State.WAIT_FRONT_ATTACH:
                self.check_front_attach_done("docking_state")
            else:
                self.check_robot_attach_done("docking_state")
            return

        if (
            self.active_scenario_id == 2
            and attached
            and self.state
            in (
                State.REAR_CART_PREP,
                State.WAIT_REAR_CART_ATTACH,
                State.REAR_CART_GRIP_SETTLE,
                State.REAR_CART_REJOIN,
            )
        ):
            self.get_logger().warn(
                f"docking_state=true ignored during {self.state.value}; "
                "waiting for scenario 2 front attach stage."
            )
            return

        if attached and self.state in (
            State.FRONT_CLEAR,
            State.REAR_ALIGN,
            State.WAIT_PRECISE_POSE,
            State.DOCK_PREP,
            State.REAR_CART_PREP,
            State.WAIT_REAR_CART_ATTACH,
            State.REAR_CART_REJOIN,
        ):
            self.get_logger().warn(
                f"docking_state=true during {self.state.value}; finishing scenario early."
            )
            self.set_cart_count(
                self.scenario_cart_count_after_docking(),
                "manual_attach",
                publish=True,
            )
            self.complete_scenario("manual attach")
            return

        if (not attached) and self.state in (State.PATROL, State.APPROACH_EXIT):
            self.abort_scenario("detached before planned detach stage")

    def schedule_front_clear(self):
        self.set_state(State.FRONT_CLEAR, "detached")

        delay = max(0.0, self.front_clear_after_detach_delay_sec)
        self.schedule_once("front_clear_start", delay, self.start_front_clear_move)
        self.get_logger().info(
            f"front clear will start after {delay:.1f}s."
        )

    def start_front_clear_move(self):
        if self.state != State.FRONT_CLEAR:
            return
        if self.is_attached:
            self.abort_scenario("front clear requested while still attached")
            return

        front_pose = self.front_initial_pose_estimate()
        if front_pose is None:
            self.front_clear_tf_retries += 1
            if self.front_clear_tf_retries > self.state_max_retries:
                self.abort_scenario("front pose unavailable for clear move")
                return
            self.get_logger().warn(
                "front clear pose unavailable. retry %d/%d"
                % (self.front_clear_tf_retries, self.state_max_retries)
            )
            self.schedule_once(
                "front_clear_tf_retry",
                self.tf_retry_delay_sec,
                self.start_front_clear_move,
            )
            return

        self.front_clear_tf_retries = 0
        x, y, yaw = front_pose
        if self.publish_front_initial_pose_before_clear:
            self.publish_front_initial_pose(x, y, yaw)

        self.get_logger().info(
            f"sending front clear odom move: {self.front_clear_distance:.2f}m forward."
        )
        if not self.send_front_clear_command():
            self.retry_or_abort("front_clear_goal_retry", self.start_front_clear_move)

    def start_scenario2_front_wait_clear(self):
        if self.state != State.FRONT_CLEAR:
            return False
        if self.active_scenario_id != 2:
            return False

        distance = self.scenario2_front_wait_clear_distance
        if distance <= 0.0:
            return False

        self.get_logger().info(
            "scenario 2 front wait clear: %.2fm forward before rear cart rejoin."
            % distance
        )
        if not self.send_front_clear_command(
            label="FRONT_WAIT_CLEAR",
            distance=distance,
            timeout_sec=self.scenario2_front_wait_clear_timeout_sec,
        ):
            self.retry_or_abort(
                "front_wait_clear_goal_retry",
                self.start_scenario2_front_wait_clear,
            )
        return True

    def front_initial_pose_estimate(self):
        source = self.front_initial_pose_source.lower()
        if source == "rear_geometry":
            rear_pose = self.robot_pose_in_map(
                ("base_footprint", "base_link", "rear_base_link")
            )
            if rear_pose is None:
                return None

            rear_x, rear_y, rear_yaw = rear_pose
            offset_x = self.front_initial_pose_rear_offset_x
            offset_y = self.front_initial_pose_rear_offset_y
            yaw = self.normalize_angle(
                rear_yaw + self.front_initial_pose_yaw_offset
            )
            x = rear_x + offset_x * math.cos(rear_yaw) - offset_y * math.sin(rear_yaw)
            y = rear_y + offset_x * math.sin(rear_yaw) + offset_y * math.cos(rear_yaw)
            return (x, y, yaw)

        return self.robot_pose_in_map(("front/base_footprint", "front/base_link"))

    def publish_front_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = self.front_initial_pose_xy_covariance
        msg.pose.covariance[7] = self.front_initial_pose_xy_covariance
        msg.pose.covariance[35] = self.front_initial_pose_yaw_covariance
        self.front_initial_pose_pub.publish(msg)
        self.get_logger().info(
            "front initial pose published: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, "
            f"source={self.front_initial_pose_source}"
        )

    def start_rear_heading_alignment(self):
        if self.state != State.REAR_ALIGN:
            self.set_state(State.REAR_ALIGN, "front_clear_done")

        if self.pending_precise_pose_msg is not None:
            self.get_logger().info(
                "precise pose already available; skipping rear heading alignment."
            )
            self.wait_for_precise_pose()
            return

        if self.detected_cart_pose is None:
            self.abort_scenario("no detected cart pose for rear alignment")
            return

        rear_pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if rear_pose is None:
            self.rear_align_tf_retries += 1
            if self.rear_align_tf_retries > self.state_max_retries:
                self.abort_scenario("rear TF unavailable for heading alignment")
                return
            self.get_logger().warn(
                "rear TF unavailable. retry %d/%d"
                % (self.rear_align_tf_retries, self.state_max_retries)
            )
            self.schedule_once(
                "rear_align_tf_retry",
                self.tf_retry_delay_sec,
                self.start_rear_heading_alignment,
            )
            return

        self.rear_align_tf_retries = 0
        rear_x, rear_y, rear_yaw = rear_pose
        cart_x = self.detected_cart_pose.pose.position.x
        cart_y = self.detected_cart_pose.pose.position.y
        yaw_to_cart = math.atan2(cart_y - rear_y, cart_x - rear_x)
        yaw_error = self.normalize_angle(yaw_to_cart - rear_yaw)

        if abs(yaw_error) <= self.rear_final_yaw_tolerance:
            self.get_logger().info(
                "rear heading already aligned: yaw_error=%.3f rad." % yaw_error
            )
            self.wait_for_precise_pose()
            return

        goal = self.create_pose_stamped(rear_x, rear_y, yaw_to_cart)
        self.get_logger().info(
            "sending rear heading goal: yaw_delta=%.3f rad." % yaw_error
        )
        if not self.send_rear_goal(goal, "REAR_ALIGN", self.diff_bt_path):
            self.retry_or_abort("rear_align_goal_retry", self.start_rear_heading_alignment)

    def wait_for_precise_pose(self):
        if self.state != State.REAR_ALIGN:
            return

        self.set_state(State.WAIT_PRECISE_POSE, "rear_align_done")
        if self.precise_pose_timeout_sec > 0.0:
            self.schedule_once(
                "precise_pose_timeout",
                self.precise_pose_timeout_sec,
                self.handle_precise_pose_timeout,
            )
        self.consume_pending_precise_pose()

    def precise_pose_callback(self, msg: PoseStamped):
        if self.state != State.WAIT_PRECISE_POSE:
            if self.state == State.REAR_ALIGN:
                self.accept_precise_pose_during_rear_align("PoseStamped")
            elif self.state == State.FRONT_CLEAR:
                self.pending_precise_pose_msg = msg
                self.pending_precise_pose_type = "PoseStamped"
                return
            else:
                return

        source_frame = self.normalize_frame_id(msg.header.frame_id or "map")
        try:
            if source_frame and source_frame != "map":
                transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
                pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            else:
                pose = msg.pose
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"precise pose TF failed: {ex}")
            return

        self.handle_precise_cart_pose(
            pose.position.x,
            pose.position.y,
            self.yaw_from_quaternion(pose.orientation),
            "PoseStamped",
        )

    def precise_pose2d_callback(self, msg: Pose2D):
        if self.state != State.WAIT_PRECISE_POSE:
            if self.state == State.REAR_ALIGN:
                self.accept_precise_pose_during_rear_align("Pose2D")
            elif self.state == State.FRONT_CLEAR:
                self.pending_precise_pose_msg = msg
                self.pending_precise_pose_type = "Pose2D"
                return
            else:
                return

        try:
            pose = self.pose2d_to_map_pose(msg)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"precise Pose2D TF failed: {ex}")
            return

        self.handle_precise_cart_pose(
            pose.position.x,
            pose.position.y,
            self.yaw_from_quaternion(pose.orientation),
            f"Pose2D/{self.precise_pose2d_frame or 'map'}",
        )

    def precise_target_point_callback(self, msg: PointStamped):
        if self.state != State.WAIT_PRECISE_POSE:
            if self.state == State.REAR_ALIGN:
                self.accept_precise_pose_during_rear_align("PointStamped")
            elif self.state == State.FRONT_CLEAR:
                self.pending_precise_pose_msg = msg
                self.pending_precise_pose_type = "PointStamped"
                return
            else:
                return

        try:
            pose = self.target_point_to_map_pose(msg)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"precise PointStamped TF failed: {ex}")
            return

        marker_x = pose.position.x
        marker_y = pose.position.y
        cart_yaw = self.yaw_from_quaternion(pose.orientation)
        aruco_id = self.parse_aruco_id(msg.header.frame_id)
        cart_x, cart_y, offset_applied = self.cart_center_from_aruco_marker(
            marker_x,
            marker_y,
            cart_yaw,
            aruco_id,
        )
        aruco_label = f"aruco_{aruco_id}" if aruco_id is not None else "aruco_unknown"
        if offset_applied:
            self.get_logger().info(
                "cart center offset from %s marker=(%.2f, %.2f, %.2f) -> "
                "center=(%.2f, %.2f)"
                % (
                    aruco_label,
                    marker_x,
                    marker_y,
                    cart_yaw,
                    cart_x,
                    cart_y,
                )
            )
        else:
            self.get_logger().warn(
                f"no cart-center offset for {aruco_label}; using marker point as cart center."
            )

        self.handle_precise_cart_pose(
            cart_x,
            cart_y,
            cart_yaw,
            f"PointStamped/{self.precise_pose2d_frame or 'map'}/{aruco_label}",
        )

    def accept_precise_pose_during_rear_align(self, source: str):
        self.get_logger().info(
            f"{source} received during rear align; starting dock prep goals now."
        )
        self.cancel_timer("rear_align_tf_retry")
        if (
            self.active_rear_goal_label == "REAR_ALIGN"
            or self.active_rear_goal_pending
            or self.active_rear_goal_handle is not None
        ):
            self.get_logger().info(
                "canceling rear heading goal because precise cart pose is available."
            )
            self.cancel_rear_goal()
        self.set_state(State.WAIT_PRECISE_POSE, f"{source}_received")
        if self.precise_pose_timeout_sec > 0.0:
            self.schedule_once(
                "precise_pose_timeout",
                self.precise_pose_timeout_sec,
                self.handle_precise_pose_timeout,
            )

    def handle_precise_cart_pose(self, cart_x, cart_y, cart_yaw, source):
        if self.state != State.WAIT_PRECISE_POSE:
            return
        if not all(math.isfinite(value) for value in (cart_x, cart_y, cart_yaw)):
            self.get_logger().warn("precise cart pose contains invalid values.")
            return

        self.cancel_timer("precise_pose_timeout")
        cart_yaw = self.normalize_angle(cart_yaw)

        if self.active_scenario_id == 2:
            self.handle_scenario2_precise_cart_pose(
                cart_x,
                cart_y,
                cart_yaw,
                source,
            )
            return

        rear_offset = self.rear_dock_goal_offset
        front_offset = self.front_dock_goal_offset
        rear_x = cart_x - rear_offset * math.cos(cart_yaw)
        rear_y = cart_y - rear_offset * math.sin(cart_yaw)
        front_x = cart_x + front_offset * math.cos(cart_yaw)
        front_y = cart_y + front_offset * math.sin(cart_yaw)

        rear_goal = self.create_pose_stamped(
            rear_x,
            rear_y,
            cart_yaw,
        )
        front_goal = self.create_pose_stamped(
            front_x,
            front_y,
            cart_yaw,
        )

        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.dock_prep_rear_goal_retry_count = 0
        self.rear_cart_prep_goal_retry_count = 0
        self.last_dock_prep_rear_goal = rear_goal
        self.last_dock_prep_front_goal = front_goal
        self.publish_dock_prep_done(False)
        self.enable_navigation_control()
        self.set_state(State.DOCK_PREP, f"precise_pose:{source}")
        self.get_logger().info(
            "dock prep goals from cart=(%.2f, %.2f, %.2f): "
            "rear=(%.2f, %.2f, %.2f, offset=%.2f), "
            "front=(%.2f, %.2f, %.2f, offset=%.2f)"
            % (
                cart_x,
                cart_y,
                cart_yaw,
                rear_goal.pose.position.x,
                rear_goal.pose.position.y,
                self.yaw_from_quaternion(rear_goal.pose.orientation),
                rear_offset,
                front_goal.pose.position.x,
                front_goal.pose.position.y,
                self.yaw_from_quaternion(front_goal.pose.orientation),
                front_offset,
            )
        )

        rear_sent = self.send_rear_goal(
            rear_goal,
            "DOCK_PREP_REAR",
            self.diff_bt_path,
        )
        front_sent = self.send_front_goal(front_goal, "DOCK_PREP_FRONT")
        if not rear_sent or not front_sent:
            self.abort_scenario("failed to send dock prep goals")
            return

        self.schedule_dock_prep_tf_check()

    def handle_scenario2_precise_cart_pose(self, cart_x, cart_y, cart_yaw, source):
        rear_offset = self.rear_dock_goal_offset
        rear_x = cart_x - rear_offset * math.cos(cart_yaw)
        rear_y = cart_y - rear_offset * math.sin(cart_yaw)
        rear_goal = self.create_pose_stamped(rear_x, rear_y, cart_yaw)

        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.dock_prep_rear_goal_retry_count = 0
        self.rear_cart_prep_goal_retry_count = 0
        self.last_dock_prep_rear_goal = rear_goal
        self.last_dock_prep_front_goal = None
        self.publish_dock_prep_done(False)
        self.enable_navigation_control()
        self.set_state(State.REAR_CART_PREP, f"precise_pose:{source}")
        self.get_logger().info(
            "scenario 2 rear cart prep from cart=(%.2f, %.2f, %.2f): "
            "rear=(%.2f, %.2f, %.2f, offset=%.2f)"
            % (
                cart_x,
                cart_y,
                cart_yaw,
                rear_goal.pose.position.x,
                rear_goal.pose.position.y,
                self.yaw_from_quaternion(rear_goal.pose.orientation),
                rear_offset,
            )
        )

        if not self.send_rear_goal(
            rear_goal,
            "REAR_CART_PREP",
            self.diff_bt_path,
        ):
            self.retry_rear_cart_prep_or_abort("failed to send rear cart prep goal")
            return

        self.schedule_dock_prep_tf_check()

    def retry_dock_prep_rear_goal(self):
        if self.state not in (State.DOCK_PREP, State.REAR_CART_PREP):
            return
        if self.state == State.REAR_CART_PREP:
            if self.accept_rear_cart_prep_by_tf("dock prep retry check"):
                return
        elif self.accept_dock_prep_goal_by_tf("rear", "dock prep retry check"):
            return
        if self.last_dock_prep_rear_goal is None:
            self.abort_scenario("no rear dock prep goal to retry")
            return
        label = "REAR_CART_PREP" if self.state == State.REAR_CART_PREP else "DOCK_PREP_REAR"
        self.get_logger().info("retrying rear dock prep goal.")
        if not self.send_rear_goal(
            self.last_dock_prep_rear_goal,
            label,
            self.diff_bt_path,
        ):
            if label == "REAR_CART_PREP":
                self.retry_rear_cart_prep_or_abort(
                    "failed to resend rear cart prep goal"
                )
            else:
                self.retry_or_abort(
                    "dock_prep_rear_goal_retry",
                    self.retry_dock_prep_rear_goal,
                )

    def retry_rear_cart_prep_or_abort(self, reason: str):
        if self.state != State.REAR_CART_PREP:
            return

        count = self.rear_cart_prep_goal_retry_count + 1
        self.rear_cart_prep_goal_retry_count = count
        max_retries = self.rear_cart_prep_goal_max_retries
        if count > max_retries:
            self.abort_scenario(
                "rear_cart_prep_goal_retry exceeded retry limit: %s" % reason
            )
            return

        delay = self.rear_cart_prep_goal_retry_delay_sec
        self.get_logger().warn(
            "%s. retry rear cart prep goal %d/%d after %.1fs"
            % (reason, count, max_retries, delay)
        )
        self.schedule_once(
            "rear_cart_prep_goal_retry",
            delay,
            self.retry_dock_prep_rear_goal,
        )

    def enter_wait_attach(self):
        self.clear_precise_pose()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.set_state(State.WAIT_ATTACH, "dock_prep_done")
        self.enable_navigation_control()

        if self.pause_rear_nav2_on_dock_prep_done:
            self.pause_rear_nav2("dock_prep_done")

        if self.auto_attach_after_dock_prep:
            self.publish_bool_burst(
                "front_robot_docking",
                self.front_robot_docking_pub,
                True,
            )
            self.publish_bool_burst("gripper_grip", self.gripper_toggle_pub, True)

        self.docking_state_confirmed = bool(self.is_attached)

        if self.attach_timeout_sec > 0.0:
            self.schedule_once(
                "attach_timeout",
                self.attach_timeout_sec,
                self.handle_attach_timeout,
            )

        self.get_logger().info(
            "dock prep complete. Waiting for /docking_state=true "
            "(RL done topics are accepted as auxiliary signals)."
        )
        self.check_rl_docking_sequence_done("docking_state_initial")

    def enter_wait_rear_cart_attach(self):
        self.cancel_rear_goal()
        self.cancel_timer("rear_cart_prep_goal_retry")
        self.clear_precise_pose()
        self.reset_rl_docking_sequence_state()
        self.rear_cart_prep_goal_retry_count = 0
        self.set_state(State.WAIT_REAR_CART_ATTACH, "rear_cart_prep_done")
        self.enable_navigation_control()
        self.rear_rl_docking_started = True
        self.publish_int_burst(
            "scenario2_rear_cart_attach_start",
            self.docking_target_pub,
            2,
        )

        if self.attach_timeout_sec > 0.0:
            self.schedule_once(
                "attach_timeout",
                self.attach_timeout_sec,
                self.handle_attach_timeout,
            )

        self.get_logger().info(
            "scenario 2 rear cart prep complete. Rear docking target start command published."
        )

    def start_rear_cart_rejoin(self):
        if self.state != State.REAR_CART_GRIP_SETTLE:
            self.get_logger().warn(
                "rear cart rejoin skipped because state=%s" % self.state.value
            )
            return

        if self.detach_pose is None:
            self.abort_scenario("no detach pose for rear cart rejoin")
            return

        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.cancel_timer("attach_timeout")
        x, y, yaw = self.detach_pose
        back_distance = self.scenario2_rear_cart_rejoin_back_distance
        goal_x = x - back_distance * math.cos(yaw)
        goal_y = y - back_distance * math.sin(yaw)
        goal = self.create_pose_stamped(goal_x, goal_y, yaw)
        self.last_rear_cart_rejoin_goal = goal
        self.set_state(State.REAR_CART_REJOIN, "rear_cart_attached")
        self.enable_navigation_control()
        self.update_dynamic_state("rear_cart_rejoin_start")
        self.get_logger().info(
            "scenario 2 rear cart rejoin goal: x=%.2f, y=%.2f, yaw=%.2f "
            "(detach=(%.2f, %.2f), back_distance=%.2f)"
            % (goal_x, goal_y, yaw, x, y, back_distance)
        )
        self.send_rear_cart_rejoin_goal(goal)

    def send_rear_cart_rejoin_goal(self, goal: PoseStamped, costmaps_prepared=False):
        if self.state != State.REAR_CART_REJOIN:
            return False

        if not costmaps_prepared and self.clear_rear_costmaps_on_cart_mode:
            self.publish_footprints()
            if self.clear_rear_costmaps("rear cart rejoin"):
                settle = max(0.0, self.rear_costmap_clear_settle_sec)
                if settle > 0.0:
                    self.get_logger().info(
                        "waiting %.2fs after rear-cart costmap clear before rejoin goal."
                        % settle
                    )
                    self.schedule_once(
                        "rear_cart_rejoin_costmap_settle",
                        settle,
                        lambda g=goal: self.send_rear_cart_rejoin_goal(
                            g,
                            costmaps_prepared=True,
                        )
                        if self.state == State.REAR_CART_REJOIN
                        else None,
                    )
                    return True

        if not self.send_rear_goal(
            goal,
            "REAR_CART_REJOIN",
            self.rear_cart_diff_bt_path,
        ):
            self.abort_scenario("failed to send rear cart rejoin goal")
            return False
        self.schedule_rear_cart_rejoin_tf_check()
        return True

    def schedule_rear_cart_rejoin_tf_check(self):
        if self.rear_cart_rejoin_tf_check_period_sec <= 0.0:
            return
        self.schedule_once(
            "rear_cart_rejoin_tf_check",
            self.rear_cart_rejoin_tf_check_period_sec,
            self.check_rear_cart_rejoin_tf_fallback,
        )

    def check_rear_cart_rejoin_tf_fallback(self):
        if self.state != State.REAR_CART_REJOIN:
            return
        if self.accept_rear_cart_rejoin_by_tf(
            "periodic TF check",
            cancel_active=True,
        ):
            return
        self.schedule_rear_cart_rejoin_tf_check()

    def accept_rear_cart_rejoin_by_tf(self, reason: str, cancel_active=False):
        if self.state != State.REAR_CART_REJOIN:
            return False
        if not self.rear_cart_rejoin_goal_reached_by_tf(reason):
            return False

        if cancel_active:
            self.cancel_rear_goal()
        self.cancel_timer("rear_cart_rejoin_tf_check")
        self.cancel_timer("rear_cart_rejoin_costmap_settle")
        self.get_logger().info(
            "rear cart rejoin accepted by TF fallback (%s)." % reason
        )
        self.enter_wait_front_attach()
        return True

    def rear_cart_rejoin_goal_reached_by_tf(self, reason: str):
        goal = self.last_rear_cart_rejoin_goal
        if goal is None:
            if reason != "periodic TF check":
                self.get_logger().warn("rear cart rejoin TF fallback unavailable: no goal.")
            return False

        robot_pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if robot_pose is None:
            if reason != "periodic TF check":
                self.get_logger().warn(
                    "rear cart rejoin TF fallback unavailable: TF lookup failed."
                )
            return False

        robot_x, robot_y, robot_yaw = robot_pose
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        goal_yaw = self.yaw_from_quaternion(goal.pose.orientation)
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        yaw_error = abs(self.normalize_angle(goal_yaw - robot_yaw))

        if (
            distance <= self.rear_cart_rejoin_tf_xy_tolerance
            and yaw_error <= self.rear_cart_rejoin_tf_yaw_tolerance
        ):
            self.get_logger().info(
                "rear cart rejoin TF fallback ok (%s): "
                "dist=%.2fm/%.2fm, yaw=%.2frad/%.2frad"
                % (
                    reason,
                    distance,
                    self.rear_cart_rejoin_tf_xy_tolerance,
                    yaw_error,
                    self.rear_cart_rejoin_tf_yaw_tolerance,
                )
            )
            return True

        if reason != "periodic TF check":
            self.get_logger().warn(
                "rear cart rejoin TF fallback rejected (%s): "
                "dist=%.2fm/%.2fm, yaw=%.2frad/%.2frad"
                % (
                    reason,
                    distance,
                    self.rear_cart_rejoin_tf_xy_tolerance,
                    yaw_error,
                    self.rear_cart_rejoin_tf_yaw_tolerance,
                )
            )
        return False

    def enter_wait_front_attach(self):
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.cancel_timer("rear_cart_rejoin_tf_check")
        self.cancel_timer("rear_cart_rejoin_costmap_settle")
        self.clear_precise_pose()
        self.scenario2_front_attach_retry_count = 0
        self.set_state(State.WAIT_FRONT_ATTACH, "rear_cart_rejoined")
        self.enable_navigation_control()
        self.docking_state_confirmed = bool(self.is_attached)
        self.get_logger().info("scenario 2 rear cart returned.")
        self.start_scenario2_front_attach_attempt("rear_cart_rejoined")

    def start_scenario2_front_attach_attempt(self, reason: str):
        if self.state != State.WAIT_FRONT_ATTACH:
            return

        self.enable_navigation_control()
        self.front_rl_docking_started = True
        self.front_rl_docking_done = False
        self.docking_state_confirmed = bool(self.is_attached)
        self.cancel_timer("scenario2_front_attach_reset")
        self.cancel_timer("scenario2_front_attach_front_home")

        if self.docking_state_confirmed:
            self.check_front_attach_done(f"{reason}:docking_state_initial")
            return

        self.publish_int_burst(
            "scenario2_front_attach_start",
            self.front_docking_target_pub,
            2,
        )

        timeout_sec = self.scenario2_front_attach_timeout_sec
        if timeout_sec <= 0.0:
            timeout_sec = self.attach_timeout_sec
        if timeout_sec > 0.0:
            self.schedule_once(
                "attach_timeout",
                timeout_sec,
                self.handle_attach_timeout,
            )

        self.get_logger().info(
            "scenario 2 front attach attempt started "
            "(retry=%d/%d, timeout=%.1fs). Waiting for /docking_state=true."
            % (
                self.scenario2_front_attach_retry_count,
                self.scenario2_front_attach_max_retries,
                timeout_sec,
            )
        )

    def retry_scenario2_front_attach(self, reason: str):
        if self.state != State.WAIT_FRONT_ATTACH or self.active_scenario_id != 2:
            return False
        if self.docking_state_confirmed:
            self.check_front_attach_done(f"{reason}:docking_state")
            return True
        if self.scenario2_front_attach_retry_count >= self.scenario2_front_attach_max_retries:
            return False

        self.scenario2_front_attach_retry_count += 1
        delay = self.scenario2_front_attach_retry_delay_sec
        self.cancel_timer("attach_timeout")
        self.front_rl_docking_started = False
        self.front_rl_docking_done = False
        self.publish_int_burst(
            "scenario2_front_attach_reset",
            self.front_docking_target_pub,
            0,
        )
        self.publish_bool_burst(
            "scenario2_front_attach_front_home",
            self.front_home_pub,
            True,
        )
        self.get_logger().warn(
            "scenario 2 front attach failed: %s. retry %d/%d after %.1fs"
            % (
                reason,
                self.scenario2_front_attach_retry_count,
                self.scenario2_front_attach_max_retries,
                delay,
            )
        )
        self.schedule_once(
            "scenario2_front_attach_retry",
            delay,
            lambda: self.start_scenario2_front_attach_attempt("front_attach_retry"),
        )
        return True

    def check_front_attach_done(self, reason: str):
        if self.state != State.WAIT_FRONT_ATTACH:
            return
        if not self.docking_state_confirmed:
            return
        if not self.front_rl_docking_done:
            self.get_logger().warn(
                "front rl_docking_done is missing, but /docking_state=true; "
                "accepting scenario 2 front attach as successful."
            )

        self.cancel_timer("attach_timeout")
        self.cancel_timer("scenario2_front_attach_retry")
        self.cancel_timer("scenario2_front_attach_start")
        self.cancel_timer("scenario2_front_attach_reset")
        self.cancel_timer("scenario2_front_attach_front_home")
        self.scenario2_front_attach_retry_count = 0
        self.set_rear_cart_attached(False, "front_attach_done")
        self.set_cart_count(
            self.scenario_cart_count_after_docking(),
            reason,
            publish=True,
        )
        if self.resume_patrol_after_docking(reason):
            return
        self.complete_scenario("front_attach_sequence_done")

    def recover_scenario_failure(self, reason: str):
        if self.scenario_recovery_active:
            return False
        if self.detach_pose is None:
            return False

        scenario1_retry_states = (
            State.FRONT_CLEAR,
            State.REAR_ALIGN,
            State.WAIT_PRECISE_POSE,
            State.DOCK_PREP,
            State.WAIT_ATTACH,
        )
        if self.active_scenario_id == 1 and self.state in scenario1_retry_states:
            if self.scenario1_pickup_retry_count >= self.scenario1_pickup_max_retries:
                self.get_logger().error(
                    "scenario 1 pickup retry limit reached: %d/%d"
                    % (
                        self.scenario1_pickup_retry_count,
                        self.scenario1_pickup_max_retries,
                    )
                )
                return False
            return self.start_scenario1_pickup_retry(reason)

        scenario2_rejoin_states = (
            State.FRONT_CLEAR,
            State.REAR_ALIGN,
            State.WAIT_PRECISE_POSE,
            State.REAR_CART_PREP,
            State.WAIT_REAR_CART_ATTACH,
            State.REAR_CART_GRIP_SETTLE,
            State.REAR_CART_REJOIN,
            State.WAIT_FRONT_ATTACH,
        )
        if self.active_scenario_id == 2 and self.state in scenario2_rejoin_states:
            return self.start_scenario2_direct_rejoin(reason)

        return False

    def start_scenario1_pickup_retry(self, reason: str):
        self.scenario1_pickup_retry_count += 1
        self.scenario_recovery_active = True
        self.rejoin_mode = "scenario1_retry"
        self.set_rear_cart_attached(False, reason)
        self.get_logger().warn(
            "scenario 1 pickup retry %d/%d after failure: %s"
            % (
                self.scenario1_pickup_retry_count,
                self.scenario1_pickup_max_retries,
                reason,
            )
        )
        return self.start_rejoin_pose_goals("scenario1_retry")

    def start_scenario2_direct_rejoin(self, reason: str):
        self.scenario_recovery_active = True
        self.rejoin_mode = "scenario2_direct_rejoin"
        self.set_rear_cart_attached(False, "scenario2_direct_rejoin")
        self.set_cart_count(0, "scenario2_direct_rejoin", publish=True)
        self.get_logger().warn(
            f"scenario 2 cart pickup abandoned. Rejoining direct ackermann: {reason}"
        )
        return self.start_rejoin_pose_goals("scenario2_direct_rejoin")

    def start_rejoin_pose_goals(self, mode: str):
        if self.detach_pose is None:
            self.scenario_recovery_active = False
            self.get_logger().error("cannot start rejoin goals: detach pose is unavailable.")
            return False

        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.last_rear_cart_rejoin_goal = None
        self.publish_dock_prep_done(False)
        self.publish_rl_docking_ready(False)
        self.clear_rl_docking_sequence_state()

        rear_goal, front_goal = self.rejoin_goal_poses()
        if rear_goal is None or front_goal is None:
            self.scenario_recovery_active = False
            return False

        self.rejoin_rear_goal_done = False
        self.rejoin_front_goal_done = False
        self.set_state(State.ROBOT_REJOIN, mode)
        self.enable_navigation_control()
        self.publish_docking_target_burst(0, f"{mode}_docking_target_reset")
        self.publish_bool_burst(f"{mode}_gripper_release", self.gripper_toggle_pub, False)
        self.publish_bool_burst(f"{mode}_front_release", self.front_home_pub, True)
        self.get_logger().info(
            "rejoin pose goals reserved (%s): rear=(%.2f, %.2f), front=(%.2f, %.2f)"
            % (
                mode,
                rear_goal.pose.position.x,
                rear_goal.pose.position.y,
                front_goal.pose.position.x,
                front_goal.pose.position.y,
            )
        )

        delay = self.recovery_release_settle_sec
        if delay > 0.0:
            self.get_logger().info(
                "waiting %.2fs for recovery release before rejoin goals." % delay
            )
            self.schedule_once(
                f"{mode}_release_settle",
                delay,
                lambda m=mode, rear=rear_goal, front=front_goal: (
                    self.send_rejoin_pose_goals_after_release(m, rear, front)
                ),
            )
            return True

        return self.send_rejoin_pose_goals_after_release(mode, rear_goal, front_goal)

    def send_rejoin_pose_goals_after_release(
        self,
        mode: str,
        rear_goal: PoseStamped,
        front_goal: PoseStamped,
    ):
        if self.state != State.ROBOT_REJOIN or self.rejoin_mode != mode:
            self.get_logger().warn(
                "rejoin goals skipped because state=%s, mode=%s"
                % (self.state.value, self.rejoin_mode)
            )
            return False

        self.enable_navigation_control()
        self.get_logger().info(
            "sending rejoin pose goals (%s): rear=(%.2f, %.2f), front=(%.2f, %.2f)"
            % (
                mode,
                rear_goal.pose.position.x,
                rear_goal.pose.position.y,
                front_goal.pose.position.x,
                front_goal.pose.position.y,
            )
        )
        rear_sent = self.send_rear_goal(rear_goal, "REJOIN_REAR", self.diff_bt_path)
        front_sent = self.send_front_goal(front_goal, "REJOIN_FRONT")
        if not rear_sent or not front_sent:
            self.scenario_recovery_active = False
            self.abort_scenario("failed to send rejoin pose goals")
            return False
        return True

    def rejoin_goal_poses(self):
        if self.detach_pose is None:
            return None, None

        x, y, yaw = self.detach_pose
        spacing = self.robot_rejoin_half_spacing
        rear_x = x - spacing * math.cos(yaw)
        rear_y = y - spacing * math.sin(yaw)
        front_x = x + spacing * math.cos(yaw)
        front_y = y + spacing * math.sin(yaw)
        return (
            self.create_pose_stamped(rear_x, rear_y, yaw),
            self.create_pose_stamped(front_x, front_y, yaw),
        )

    def check_rejoin_pose_done(self):
        if self.state != State.ROBOT_REJOIN:
            return
        if not (self.rejoin_rear_goal_done and self.rejoin_front_goal_done):
            return

        mode = self.rejoin_mode
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.rejoin_rear_goal_done = False
        self.rejoin_front_goal_done = False

        if mode == "scenario1_retry":
            self.scenario_recovery_active = False
            self.rejoin_mode = None
            self.clear_precise_pose()
            self.get_logger().info(
                "scenario 1 robots returned to detach area. Restarting rear heading alignment."
            )
            self.start_rear_heading_alignment()
            return

        if mode == "scenario2_direct_rejoin":
            self.enter_wait_robot_attach()
            return

        self.abort_scenario(f"unknown rejoin mode: {mode}")

    def enter_wait_robot_attach(self):
        self.clear_precise_pose()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.reset_rl_docking_sequence_state()
        self.set_state(State.WAIT_ROBOT_ATTACH, "direct_robot_rejoin_pose_done")
        self.enable_navigation_control()
        self.front_rl_docking_started = True
        self.rear_rl_docking_started = True
        self.publish_front_docking_target(1)
        self.publish_rear_docking_target(1)

        if self.attach_timeout_sec > 0.0:
            self.schedule_once(
                "attach_timeout",
                self.attach_timeout_sec,
                self.handle_attach_timeout,
            )

        self.get_logger().info(
            "direct robot rejoin targets published. Waiting for /docking_state=true "
            "(RL done topics are accepted as auxiliary signals)."
        )
        self.check_robot_attach_done("docking_state_initial")

    def check_robot_attach_done(self, reason: str):
        if self.state != State.WAIT_ROBOT_ATTACH:
            return
        if not self.docking_state_confirmed:
            return
        if not (self.front_rl_docking_done and self.rear_rl_docking_done):
            self.get_logger().warn(
                "robot rejoin RL done topic is missing "
                f"(front_done={self.front_rl_docking_done}, "
                f"rear_done={self.rear_rl_docking_done}), "
                "but /docking_state=true; accepting robot rejoin as successful."
            )

        self.cancel_timer("attach_timeout")
        self.scenario_recovery_active = False
        self.rejoin_mode = None
        self.set_rear_cart_attached(False, reason)
        self.set_cart_count(0, reason, publish=True)
        if self.resume_patrol_after_docking(reason):
            return
        self.complete_scenario("direct_robot_rejoin_done")

    # ------------------------------------------------------------------
    # Route and action callbacks
    # ------------------------------------------------------------------
    def start_route(self, points, route_type, final_yaw=None, patrol_start_index=0):
        poses = self.create_path_poses(points, final_yaw)
        if not poses:
            if route_type == "EXIT":
                self.start_detach_sequence()
                return True
            self.abort_scenario("route has no valid poses")
            return False

        self.active_route_type = route_type
        self.active_route_poses = poses
        self.active_route_index = 0
        self.active_route_patrol_start_index = (
            max(0, int(patrol_start_index)) if route_type == "PATROL" else 0
        )
        self.route_goal_retry_count = 0
        return self.send_current_route_goal()

    def send_current_route_goal(self, costmaps_prepared=False):
        if self.active_route_index >= len(self.active_route_poses):
            self.finish_route()
            return True

        if (
            not costmaps_prepared
            and self.prepare_cart_mode_route_goal_costmaps()
        ):
            self.schedule_once(
                "route_goal_costmap_settle",
                self.rear_costmap_clear_settle_sec,
                lambda: self.send_current_route_goal(costmaps_prepared=True)
                if self.active_route_type is not None
                else None,
            )
            return True

        pose = self.active_route_poses[self.active_route_index]
        pose.header.stamp = self.get_clock().now().to_msg()
        self.update_dynamic_state("route_goal")

        label = f"ROUTE_{self.active_route_type}"
        self.get_logger().info(
            "route %s goal %d/%d"
            % (
                self.active_route_type,
                self.active_route_index + 1,
                len(self.active_route_poses),
            )
        )

        if not self.send_rear_goal(pose, label, self.current_behavior_tree()):
            self.handle_route_failure("failed to send route goal")
            return False
        self.schedule_route_tf_check()
        return True

    def finish_route(self):
        route_type = self.active_route_type
        self.clear_route()

        if route_type == "PATROL":
            self.complete_scenario("patrol complete")
        elif route_type == "EXIT":
            self.prepare_patrol_resume_after_docking(
                "exit_reached",
                use_active_route=False,
            )
            self.start_detach_sequence()

    def send_rear_goal(self, pose: PoseStamped, label: str, behavior_tree: str):
        self.cancel_timer("rear_nav_control")
        self.publish_bool_once(self.rear_joy_sig_pub, False)
        if not self.rear_nav_client.wait_for_server(timeout_sec=self.nav_server_timeout_sec):
            self.get_logger().error("rear navigate_to_pose server is not ready.")
            return False

        self.active_rear_goal_seq += 1
        seq = self.active_rear_goal_seq
        self.active_rear_goal_pending = True
        self.active_rear_goal_label = label

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.behavior_tree = behavior_tree

        try:
            future = self.rear_nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda fut, goal_seq=seq: self.rear_goal_response(fut, goal_seq)
            )
        except Exception as ex:
            self.active_rear_goal_pending = False
            self.get_logger().error(f"rear goal send exception: {ex}")
            return False
        return True

    def send_front_goal(self, pose: PoseStamped, label: str):
        if self.front_nav_transport == "topic_proxy":
            return self.send_front_proxy_goal(pose, label)

        if not self.front_nav_client.wait_for_server(timeout_sec=self.nav_server_timeout_sec):
            self.get_logger().error("front navigate_to_pose server is not ready.")
            return False

        self.active_front_goal_seq += 1
        seq = self.active_front_goal_seq
        self.active_front_goal_pending = True
        self.active_front_goal_label = label

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        try:
            future = self.front_nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda fut, goal_seq=seq: self.front_goal_response(fut, goal_seq)
            )
        except Exception as ex:
            self.active_front_goal_pending = False
            self.get_logger().error(f"front goal send exception: {ex}")
            return False
        self.schedule_front_goal_timeout()
        return True

    def send_front_clear_command(
        self,
        label="FRONT_CLEAR",
        distance=None,
        speed=None,
        timeout_sec=None,
    ):
        if self.front_nav_transport != "topic_proxy":
            self.get_logger().error(
                "front clear odom move requires front_nav_transport=topic_proxy."
            )
            return False

        distance = self.front_clear_distance if distance is None else float(distance)
        distance = max(0.0, distance)
        speed = self.front_clear_speed if speed is None else abs(float(speed))
        speed = max(0.01, speed)
        timeout_sec = (
            self.front_clear_timeout_sec if timeout_sec is None else float(timeout_sec)
        )

        self.publish_bool_once(self.front_joy_sig_pub, False)
        self.active_front_goal_seq += 1
        goal_id = self.active_front_goal_seq
        self.active_front_proxy_goal_id = goal_id
        self.active_front_goal_pending = True
        self.active_front_goal_label = label

        if timeout_sec <= 0.0:
            timeout_sec = max(3.0, distance / speed + 3.0)

        msg = String()
        msg.data = json.dumps(
            {
                "id": goal_id,
                "label": label,
                "command": "clear_forward",
                "distance": float(distance),
                "speed": float(speed),
                "timeout_sec": float(timeout_sec),
            }
        )
        self.front_nav_goal_pub.publish(msg)
        self.get_logger().info(
            "front clear command published: "
            f"id={goal_id}, label={label}, distance={distance:.2f}, "
            f"speed={speed:.2f}"
        )
        self.schedule_front_goal_timeout()
        return True

    def send_front_proxy_goal(self, pose: PoseStamped, label: str):
        self.active_front_goal_seq += 1
        goal_id = self.active_front_goal_seq
        self.active_front_proxy_goal_id = goal_id
        self.active_front_goal_pending = True
        self.active_front_goal_label = label

        msg = String()
        msg.data = json.dumps(
            {
                "id": goal_id,
                "label": label,
                "frame_id": pose.header.frame_id or "map",
                "stamp": {
                    "sec": int(pose.header.stamp.sec),
                    "nanosec": int(pose.header.stamp.nanosec),
                },
                "position": {
                    "x": float(pose.pose.position.x),
                    "y": float(pose.pose.position.y),
                    "z": float(pose.pose.position.z),
                },
                "orientation": {
                    "x": float(pose.pose.orientation.x),
                    "y": float(pose.pose.orientation.y),
                    "z": float(pose.pose.orientation.z),
                    "w": float(pose.pose.orientation.w),
                },
            }
        )
        self.front_nav_goal_pub.publish(msg)
        self.get_logger().info(
            f"front proxy goal published: id={goal_id}, label={label}"
        )
        self.schedule_front_goal_timeout()
        return True

    def front_nav_proxy_result_callback(self, msg: String):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError as ex:
            self.get_logger().warn(f"front proxy result JSON parse failed: {ex}")
            return

        goal_id = int(result.get("id", -1))
        if goal_id != self.active_front_proxy_goal_id:
            self.get_logger().warn(
                "stale front proxy result ignored: "
                f"id={goal_id}, active={self.active_front_proxy_goal_id}"
            )
            return

        if not self.active_front_goal_pending:
            return

        self.cancel_timer("front_goal_timeout")
        self.active_front_goal_pending = False
        label = self.active_front_goal_label
        success = bool(result.get("success", False))
        if success:
            self.handle_front_goal_success(label)
            return

        reason = str(result.get("message", "front proxy goal failed"))
        self.handle_front_goal_failure(reason)

    def rear_goal_response(self, future: Future, goal_seq: int):
        if goal_seq != self.active_rear_goal_seq:
            return
        self.active_rear_goal_pending = False

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.handle_rear_goal_failure(f"rear goal response exception: {ex}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.handle_rear_goal_failure("rear goal rejected")
            return

        self.get_logger().info(f"rear goal accepted: {self.active_rear_goal_label}")
        self.active_rear_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, goal_seq=goal_seq: self.rear_goal_result(fut, goal_seq)
        )

    def front_goal_response(self, future: Future, goal_seq: int):
        if goal_seq != self.active_front_goal_seq:
            return
        self.active_front_goal_pending = False

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.handle_front_goal_failure(f"front goal response exception: {ex}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.handle_front_goal_failure("front goal rejected")
            return

        self.active_front_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, goal_seq=goal_seq: self.front_goal_result(fut, goal_seq)
        )

    def rear_goal_result(self, future: Future, goal_seq: int):
        if goal_seq != self.active_rear_goal_seq:
            return

        label = self.active_rear_goal_label
        self.active_rear_goal_handle = None
        try:
            result = future.result()
        except Exception as ex:
            self.handle_rear_goal_failure(f"rear goal result exception: {ex}")
            return

        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = "None" if result is None else str(result.status)
            self.handle_rear_goal_failure(f"rear goal failed status={status}")
            return

        self.get_logger().info(f"rear goal succeeded: {label}")
        if label in ("ROUTE_PATROL", "ROUTE_EXIT"):
            if self.active_route_type not in ("PATROL", "EXIT"):
                self.get_logger().info(
                    "rear route result ignored because active_route_type=%s"
                    % self.active_route_type
                )
                return
            self.cancel_timer("route_tf_check")
            if not self.route_goal_reached_by_tf():
                self.handle_route_failure("route waypoint TF verification failed")
                return
            if self.active_route_type == "PATROL":
                completed_patrol_index = (
                    self.active_route_patrol_start_index
                    + self.active_route_index
                    + 1
                )
                self.current_patrol_waypoint_index = max(
                    self.current_patrol_waypoint_index,
                    completed_patrol_index,
                )
            self.route_goal_retry_count = 0
            self.active_route_index += 1
            self.send_current_route_goal()
        elif label == "REAR_ALIGN":
            if self.state != State.REAR_ALIGN:
                self.get_logger().info(
                    "rear align result ignored because state=%s" % self.state.value
                )
                return
            self.wait_for_precise_pose()
        elif label == "DOCK_PREP_REAR":
            if self.state != State.DOCK_PREP:
                self.get_logger().info(
                    "rear dock prep result ignored because state=%s"
                    % self.state.value
                )
                return
            self.rear_dock_goal_done = True
            self.check_dock_prep_done()
        elif label == "REAR_CART_PREP":
            if self.state != State.REAR_CART_PREP:
                self.get_logger().info(
                    "rear cart prep result ignored because state=%s"
                    % self.state.value
                )
                return
            self.enter_wait_rear_cart_attach()
        elif label == "REAR_CART_REJOIN":
            if self.state != State.REAR_CART_REJOIN:
                self.get_logger().info(
                    "rear cart rejoin result ignored because state=%s"
                    % self.state.value
                )
                return
            self.cancel_timer("rear_cart_rejoin_tf_check")
            self.enter_wait_front_attach()
        elif label == "REJOIN_REAR":
            if self.state != State.ROBOT_REJOIN:
                self.get_logger().info(
                    "rear robot rejoin result ignored because state=%s"
                    % self.state.value
                )
                return
            self.rejoin_rear_goal_done = True
            self.check_rejoin_pose_done()
        elif label == "MANUAL":
            self.set_state(State.IDLE, "manual_goal_done")

    def front_goal_result(self, future: Future, goal_seq: int):
        if goal_seq != self.active_front_goal_seq:
            return

        self.cancel_timer("front_goal_timeout")
        label = self.active_front_goal_label
        self.active_front_goal_handle = None
        try:
            result = future.result()
        except Exception as ex:
            self.handle_front_goal_failure(f"front goal result exception: {ex}")
            return

        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = "None" if result is None else str(result.status)
            self.handle_front_goal_failure(f"front goal failed status={status}")
            return

        self.handle_front_goal_success(label)

    def handle_front_goal_success(self, label):
        if label == "FRONT_CLEAR":
            if self.state != State.FRONT_CLEAR:
                self.get_logger().info(
                    "front clear result ignored because state=%s" % self.state.value
                )
                return
            if self.start_scenario2_front_wait_clear():
                return
            self.start_rear_heading_alignment()
        elif label == "FRONT_WAIT_CLEAR":
            if self.state != State.FRONT_CLEAR:
                self.get_logger().info(
                    "front wait clear result ignored because state=%s"
                    % self.state.value
                )
                return
            self.start_rear_heading_alignment()
        elif label == "DOCK_PREP_FRONT":
            if self.state != State.DOCK_PREP:
                self.get_logger().info(
                    "front dock prep result ignored because state=%s"
                    % self.state.value
                )
                return
            self.front_dock_goal_done = True
            self.check_dock_prep_done()
        elif label == "REJOIN_FRONT":
            if self.state != State.ROBOT_REJOIN:
                self.get_logger().info(
                    "front rejoin result ignored because state=%s"
                    % self.state.value
                )
                return
            self.rejoin_front_goal_done = True
            self.check_rejoin_pose_done()

    def check_dock_prep_done(self):
        if (
            self.state == State.DOCK_PREP
            and self.rear_dock_goal_done
            and self.front_dock_goal_done
        ):
            self.cancel_timer("dock_prep_tf_check")
            self.publish_dock_prep_done(True)
            self.publish_rl_docking_ready(True)
            self.reset_rl_docking_sequence_state()
            self.enter_wait_attach()
            if self.state == State.WAIT_ATTACH:
                self.front_rl_docking_started = True
                self.publish_int_burst(
                    "scenario1_front_attach_start",
                    self.front_docking_target_pub,
                    2,
                )
                self.get_logger().info(
                    "front docking target start command published."
                )

    def schedule_dock_prep_tf_check(self):
        if self.dock_prep_tf_check_period_sec <= 0.0:
            return
        self.schedule_once(
            "dock_prep_tf_check",
            self.dock_prep_tf_check_period_sec,
            self.check_dock_prep_tf_fallback,
        )

    def check_dock_prep_tf_fallback(self):
        if self.state == State.REAR_CART_PREP:
            if self.accept_rear_cart_prep_by_tf(
                "periodic TF check",
                cancel_active=True,
            ):
                return
            self.schedule_dock_prep_tf_check()
            return

        if self.state != State.DOCK_PREP:
            return

        if not self.rear_dock_goal_done:
            self.accept_dock_prep_goal_by_tf(
                "rear",
                "periodic TF check",
                cancel_active=True,
            )
        if self.state != State.DOCK_PREP:
            return

        if not self.front_dock_goal_done:
            self.accept_dock_prep_goal_by_tf(
                "front",
                "periodic TF check",
                cancel_active=True,
            )
        if self.state == State.DOCK_PREP:
            self.schedule_dock_prep_tf_check()

    def accept_dock_prep_goal_by_tf(
        self,
        robot_name: str,
        reason: str,
        cancel_active=False,
    ):
        if self.state != State.DOCK_PREP:
            return False
        if not self.dock_prep_goal_reached_by_tf(
            robot_name,
            reason,
            allow_start_tolerance=True,
        ):
            return False

        if cancel_active:
            if robot_name == "front":
                self.cancel_front_goal()
            elif robot_name == "rear":
                self.cancel_rear_goal()

        if robot_name == "front":
            self.front_dock_goal_done = True
            self.active_front_goal_pending = False
            self.active_front_goal_label = None
            self.active_front_proxy_goal_id = 0
        elif robot_name == "rear":
            self.rear_dock_goal_done = True
            self.active_rear_goal_pending = False
            self.active_rear_goal_label = None
        else:
            return False

        self.get_logger().info(
            f"{robot_name} dock prep accepted by TF fallback ({reason})."
        )
        self.check_dock_prep_done()
        return True

    def accept_all_dock_prep_goals_by_tf(
        self,
        reason: str,
        cancel_active=False,
        required_robot=None,
    ):
        if self.state != State.DOCK_PREP:
            return False

        if not self.rear_dock_goal_done:
            self.accept_dock_prep_goal_by_tf(
                "rear",
                reason,
                cancel_active=cancel_active,
            )
        if self.state != State.DOCK_PREP:
            return True

        if not self.front_dock_goal_done:
            self.accept_dock_prep_goal_by_tf(
                "front",
                reason,
                cancel_active=cancel_active,
            )

        if self.state != State.DOCK_PREP:
            return True
        if required_robot == "rear":
            return self.rear_dock_goal_done
        if required_robot == "front":
            return self.front_dock_goal_done
        return self.rear_dock_goal_done or self.front_dock_goal_done

    def accept_rear_cart_prep_by_tf(self, reason: str, cancel_active=False):
        if self.state != State.REAR_CART_PREP:
            return False
        if not self.dock_prep_goal_reached_by_tf(
            "rear",
            reason,
            allow_start_tolerance=False,
        ):
            return False

        if cancel_active:
            self.cancel_rear_goal()

        self.rear_dock_goal_done = True
        self.active_rear_goal_pending = False
        self.active_rear_goal_label = None
        self.get_logger().info(
            f"rear cart prep accepted by TF fallback ({reason})."
        )
        self.enter_wait_rear_cart_attach()
        return True

    def dock_prep_goal_reached_by_tf(
        self,
        robot_name: str,
        reason: str,
        allow_start_tolerance=False,
    ):
        if robot_name == "front":
            goal = self.last_dock_prep_front_goal
            frame_candidates = ("front/base_footprint", "front/base_link")
        elif robot_name == "rear":
            goal = self.last_dock_prep_rear_goal
            frame_candidates = ("base_footprint", "base_link", "rear_base_link")
        else:
            return False

        if goal is None:
            if reason != "periodic TF check":
                self.get_logger().warn(
                    f"{robot_name} dock prep TF fallback unavailable: no stored goal."
                )
            return False

        robot_pose = self.robot_pose_in_map(frame_candidates)
        if robot_pose is None:
            if reason != "periodic TF check":
                self.get_logger().warn(
                    f"{robot_name} dock prep TF fallback unavailable: TF lookup failed."
                )
            return False

        robot_x, robot_y, robot_yaw = robot_pose
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        goal_yaw = self.yaw_from_quaternion(goal.pose.orientation)
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        yaw_error = abs(self.normalize_angle(goal_yaw - robot_yaw))

        strict_ok = (
            distance <= self.dock_prep_tf_xy_tolerance
            and yaw_error <= self.dock_prep_tf_yaw_tolerance
        )
        start_ok = (
            allow_start_tolerance
            and distance <= self.dock_prep_start_xy_tolerance
            and yaw_error <= self.dock_prep_start_yaw_tolerance
        )

        if strict_ok or start_ok:
            tolerance_mode = "strict" if strict_ok else "start"
            xy_tolerance = (
                self.dock_prep_tf_xy_tolerance
                if strict_ok
                else self.dock_prep_start_xy_tolerance
            )
            yaw_tolerance = (
                self.dock_prep_tf_yaw_tolerance
                if strict_ok
                else self.dock_prep_start_yaw_tolerance
            )
            self.get_logger().info(
                "%s dock prep TF fallback ok (%s, %s): "
                "dist=%.2fm/%.2fm, yaw=%.2frad/%.2frad"
                % (
                    robot_name,
                    reason,
                    tolerance_mode,
                    distance,
                    xy_tolerance,
                    yaw_error,
                    yaw_tolerance,
                )
            )
            return True

        if reason != "periodic TF check":
            xy_tolerance = (
                self.dock_prep_start_xy_tolerance
                if allow_start_tolerance
                else self.dock_prep_tf_xy_tolerance
            )
            yaw_tolerance = (
                self.dock_prep_start_yaw_tolerance
                if allow_start_tolerance
                else self.dock_prep_tf_yaw_tolerance
            )
            self.get_logger().warn(
                "%s dock prep TF fallback rejected (%s): dist=%.2fm/%.2fm, yaw=%.2frad/%.2frad"
                % (
                    robot_name,
                    reason,
                    distance,
                    xy_tolerance,
                    yaw_error,
                    yaw_tolerance,
                )
            )
        return False

    def reset_rl_docking_sequence_state(self):
        self.front_rl_docking_started = False
        self.front_rl_docking_done = False
        self.rear_rl_docking_done = False
        self.rear_rl_docking_started = False
        self.docking_state_confirmed = bool(self.is_attached)

    def clear_rl_docking_sequence_state(self):
        self.front_rl_docking_started = False
        self.front_rl_docking_done = False
        self.rear_rl_docking_done = False
        self.rear_rl_docking_started = False
        self.docking_state_confirmed = bool(self.is_attached)

    def check_rl_docking_sequence_done(self, reason: str):
        if self.state != State.WAIT_ATTACH:
            return

        if not self.docking_state_confirmed:
            return
        if not (self.front_rl_docking_done and self.rear_rl_docking_done):
            self.get_logger().warn(
                "cart attach RL done topic is missing "
                f"(front_done={self.front_rl_docking_done}, "
                f"rear_done={self.rear_rl_docking_done}), "
                "but /docking_state=true; accepting cart attach as successful."
            )

        self.cancel_timer("attach_timeout")
        self.set_cart_count(
            self.scenario_cart_count_after_docking(),
            reason,
            publish=True,
        )
        if self.resume_patrol_after_docking(reason):
            return
        self.complete_scenario("rl_docking_sequence_done")

    def clear_patrol_resume_after_docking(self):
        self.pending_patrol_resume_after_docking = False
        self.resume_patrol_waypoint_index = 0

    def prepare_patrol_resume_after_docking(self, reason: str, use_active_route: bool):
        if not self.patrol_path:
            self.clear_patrol_resume_after_docking()
            return

        if (
            use_active_route
            and self.active_route_type == "PATROL"
            and self.active_route_poses
        ):
            index = self.active_route_patrol_start_index + self.active_route_index
            index = max(index, self.current_patrol_waypoint_index)
        else:
            progress = self.current_progress_on_path()
            index = self.next_patrol_waypoint_index_after_progress(progress)
            index = max(index, self.current_patrol_waypoint_index)

        index = max(0, min(int(index), len(self.patrol_path)))
        if index >= len(self.patrol_path):
            self.clear_patrol_resume_after_docking()
            self.get_logger().info(
                f"no remaining patrol waypoint to resume after docking ({reason})."
            )
            return

        self.pending_patrol_resume_after_docking = True
        self.resume_patrol_waypoint_index = index
        self.get_logger().info(
            "patrol resume reserved after docking: waypoint %d/%d (%s)"
            % (index + 1, len(self.patrol_path), reason)
        )

    def resume_patrol_after_docking(self, reason: str):
        if not self.pending_patrol_resume_after_docking:
            return False

        index = max(
            0,
            min(int(self.resume_patrol_waypoint_index), len(self.patrol_path)),
        )
        remaining = self.patrol_path[index:]
        if not remaining:
            self.clear_patrol_resume_after_docking()
            self.get_logger().info("patrol route already complete after docking.")
            return False

        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.last_rear_cart_rejoin_goal = None
        self.publish_dock_prep_done(False)
        self.publish_rl_docking_ready(False)
        self.clear_rl_docking_sequence_state()
        self.publish_docking_target_burst(0, "resume_after_docking_target_reset")
        self.enable_navigation_control()
        self.current_patrol_waypoint_index = max(
            self.current_patrol_waypoint_index,
            index,
        )
        self.clear_patrol_resume_after_docking()

        final_yaw = None
        if len(remaining) == 1:
            final_yaw = self.patrol_waypoint_yaw(index)

        wait_for_resume = (
            self.resume_rear_nav2_on_scenario_start and self.rear_nav2_paused
        )
        if self.resume_rear_nav2_on_scenario_start:
            self.resume_rear_nav2("resume_after_docking")

        self.set_state(State.PATROL, f"resume_after_docking:{reason}")
        self.get_logger().info(
            "resuming patrol after docking from waypoint %d/%d"
            % (index + 1, len(self.patrol_path))
        )

        if wait_for_resume:
            self.schedule_once(
                "rear_nav2_resume_after_docking",
                self.rear_nav2_resume_delay_sec,
                lambda route=remaining, yaw=final_yaw, start=index: self.start_route(
                    route,
                    "PATROL",
                    final_yaw=yaw,
                    patrol_start_index=start,
                )
                if self.state == State.PATROL
                else None,
            )
            return True

        self.start_route(
            remaining,
            "PATROL",
            final_yaw=final_yaw,
            patrol_start_index=index,
        )
        return True

    def handle_rear_goal_failure(self, reason):
        label = self.active_rear_goal_label
        self.active_rear_goal_handle = None
        self.active_rear_goal_pending = False

        if label in ("ROUTE_PATROL", "ROUTE_EXIT"):
            if self.active_route_type not in ("PATROL", "EXIT"):
                self.get_logger().info(
                    "rear route failure ignored because active_route_type=%s: %s"
                    % (self.active_route_type, reason)
                )
                return
            if self.accept_current_route_goal_by_tf(reason, cancel_active=False):
                return
            self.handle_route_failure(reason)
        elif label == "REAR_ALIGN":
            if self.state != State.REAR_ALIGN:
                self.get_logger().info(
                    "rear align failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            self.get_logger().warn(reason)
            self.retry_or_abort("rear_align_goal_retry", self.start_rear_heading_alignment)
        elif label == "DOCK_PREP_REAR":
            if self.state != State.DOCK_PREP:
                self.get_logger().info(
                    "rear dock prep failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            if self.accept_all_dock_prep_goals_by_tf(
                reason,
                cancel_active=False,
                required_robot="rear",
            ):
                return
            self.get_logger().warn(reason)
            self.retry_or_abort("dock_prep_rear_goal_retry", self.retry_dock_prep_rear_goal)
        elif label == "REAR_CART_PREP":
            if self.state != State.REAR_CART_PREP:
                self.get_logger().info(
                    "rear cart prep failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            if self.accept_rear_cart_prep_by_tf(reason):
                return
            self.get_logger().warn(reason)
            self.retry_rear_cart_prep_or_abort(reason)
        elif label == "REAR_CART_REJOIN":
            if self.state != State.REAR_CART_REJOIN:
                self.get_logger().info(
                    "rear cart rejoin failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            if self.accept_rear_cart_rejoin_by_tf(reason, cancel_active=False):
                return
            self.abort_scenario(reason)
        elif label == "REJOIN_REAR":
            if self.state != State.ROBOT_REJOIN:
                self.get_logger().info(
                    "rear robot rejoin failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            self.abort_scenario(reason)
        elif label == "MANUAL":
            self.abort_scenario(reason)
        else:
            self.get_logger().warn(reason)

    def handle_front_goal_failure(self, reason):
        self.cancel_timer("front_goal_timeout")
        label = self.active_front_goal_label
        self.active_front_goal_handle = None
        self.active_front_goal_pending = False
        self.active_front_proxy_goal_id = 0

        if label == "FRONT_CLEAR":
            if self.state != State.FRONT_CLEAR:
                self.get_logger().info(
                    "front clear failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            self.retry_or_abort("front_clear_goal_retry", self.start_front_clear_move)
        elif label == "FRONT_WAIT_CLEAR":
            if self.state != State.FRONT_CLEAR:
                self.get_logger().info(
                    "front wait clear failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            self.retry_or_abort(
                "front_wait_clear_goal_retry",
                self.start_scenario2_front_wait_clear,
            )
        elif label == "DOCK_PREP_FRONT":
            if self.state != State.DOCK_PREP:
                self.get_logger().info(
                    "front dock prep failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            if self.accept_all_dock_prep_goals_by_tf(
                reason,
                cancel_active=False,
                required_robot="front",
            ):
                return
            self.abort_scenario(reason)
        elif label == "REJOIN_FRONT":
            if self.state != State.ROBOT_REJOIN:
                self.get_logger().info(
                    "front rejoin failure ignored because state=%s: %s"
                    % (self.state.value, reason)
                )
                return
            self.abort_scenario(reason)
        else:
            self.get_logger().warn(reason)

    def schedule_front_goal_timeout(self):
        timeout_sec = self.front_goal_timeout_sec
        if self.active_front_goal_label == "DOCK_PREP_FRONT":
            timeout_sec = self.dock_prep_front_goal_timeout_sec

        if timeout_sec <= 0.0:
            return
        self.schedule_once(
            "front_goal_timeout",
            timeout_sec,
            self.handle_front_goal_timeout,
        )

    def handle_front_goal_timeout(self):
        if self.active_front_goal_label is None:
            return

        if self.active_front_goal_label == "DOCK_PREP_FRONT":
            if self.accept_all_dock_prep_goals_by_tf(
                "front goal timeout",
                cancel_active=True,
                required_robot="front",
            ):
                return

        proxy_goal_id = self.active_front_proxy_goal_id
        if self.front_nav_transport == "topic_proxy" and proxy_goal_id:
            msg = String()
            msg.data = json.dumps({"id": proxy_goal_id, "cancel": True})
            self.front_nav_cancel_pub.publish(msg)
        elif self.active_front_goal_handle is not None:
            self.active_front_goal_handle.cancel_goal_async()

        self.handle_front_goal_failure("front goal timeout")

    def handle_route_failure(self, reason):
        self.route_goal_retry_count += 1
        goal_was_rejected = "rejected" in str(reason).lower()
        max_retries = (
            max(self.route_goal_max_retries, self.route_goal_rejected_max_retries)
            if goal_was_rejected
            else self.route_goal_max_retries
        )
        retry_delay = (
            max(
                self.route_goal_retry_delay_sec,
                self.route_goal_rejected_retry_delay_sec,
            )
            if goal_was_rejected
            else self.route_goal_retry_delay_sec
        )

        if goal_was_rejected:
            self.resume_rear_nav2("route_goal_rejected")

        if self.route_goal_retry_count > max_retries:
            if self.active_route_type == "PATROL":
                self.skip_current_patrol_waypoint(reason)
                return
            self.abort_scenario(reason)
            return

        self.get_logger().warn(
            "%s. retry route goal %d/%d"
            % (reason, self.route_goal_retry_count, max_retries)
        )
        self.schedule_once(
            "route_goal_retry",
            retry_delay,
            self.send_current_route_goal,
        )

    def skip_current_patrol_waypoint(self, reason: str):
        if self.active_route_type != "PATROL":
            self.abort_scenario(reason)
            return
        if self.active_route_index >= len(self.active_route_poses):
            self.finish_route()
            return

        route_index = self.active_route_index
        route_count = len(self.active_route_poses)
        completed_patrol_index = self.active_route_patrol_start_index + route_index + 1
        self.cancel_timer("route_tf_check")
        self.cancel_timer("route_goal_retry")
        self.current_patrol_waypoint_index = max(
            self.current_patrol_waypoint_index,
            completed_patrol_index,
        )
        self.active_route_index = route_index + 1
        self.route_goal_retry_count = 0
        self.get_logger().warn(
            "route PATROL goal %d/%d skipped after retry limit: %s"
            % (route_index + 1, route_count, reason)
        )
        self.send_current_route_goal()

    def retry_or_abort(self, key, callback):
        count_key = f"{key}_count"
        count = getattr(self, count_key, 0) + 1
        setattr(self, count_key, count)
        if count > self.state_max_retries:
            self.abort_scenario(f"{key} exceeded retry limit")
            return

        self.get_logger().warn(
            "%s retry %d/%d" % (key, count, self.state_max_retries)
        )
        self.schedule_once(key, self.route_goal_retry_delay_sec, callback)

    # ------------------------------------------------------------------
    # Timeouts and scenario completion
    # ------------------------------------------------------------------
    def handle_detach_timeout(self):
        if self.state == State.WAIT_DETACH:
            reason = (
                "waiting for docking_state=false "
                f"(is_attached={self.is_attached})"
            )
            if self.retry_detach_release(reason):
                return
            self.abort_scenario(
                "detach timeout after release retries: "
                f"{reason}, "
                f"retries={self.detach_release_retry_count}/"
                f"{self.detach_release_max_retries}"
            )

    def handle_precise_pose_timeout(self):
        if self.state == State.WAIT_PRECISE_POSE:
            self.abort_scenario(
                "precise pose timeout "
                f"(scenario={self.active_scenario_id}, "
                f"detected_cart={self.detected_cart_pose is not None})"
            )

    def handle_attach_timeout(self):
        if self.state in (
            State.WAIT_ATTACH,
            State.WAIT_REAR_CART_ATTACH,
            State.WAIT_FRONT_ATTACH,
            State.WAIT_ROBOT_ATTACH,
        ):
            if self.state == State.WAIT_ATTACH and self.docking_state_confirmed:
                self.check_rl_docking_sequence_done("attach_timeout:docking_state")
                return
            if self.state == State.WAIT_FRONT_ATTACH:
                if self.docking_state_confirmed:
                    self.check_front_attach_done("attach_timeout:docking_state")
                    return
                if self.retry_scenario2_front_attach("attach timeout"):
                    return
            if self.state == State.WAIT_ROBOT_ATTACH and self.docking_state_confirmed:
                self.check_robot_attach_done("attach_timeout:docking_state")
                return

            self.abort_scenario(
                "attach timeout waiting for expected rl_docking_done and "
                "docking_state: "
                f"state={self.state.value}, "
                f"front_started={self.front_rl_docking_started}, "
                f"front_done={self.front_rl_docking_done}, "
                f"rear_started={self.rear_rl_docking_started}, "
                f"rear_done={self.rear_rl_docking_done}, "
                f"docking_state_confirmed={self.docking_state_confirmed}, "
                f"is_attached={self.is_attached}"
            )

    def complete_scenario(self, reason):
        self.get_logger().info(f"scenario complete: {reason}")
        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.last_rear_cart_rejoin_goal = None
        self.publish_dock_prep_done(False)
        self.publish_rl_docking_ready(False)
        self.clear_rl_docking_sequence_state()
        self.publish_docking_target_burst(0, "scenario_complete_target_reset")
        self.clear_patrol_resume_after_docking()
        self.active_scenario_id = 0
        self.detach_pose = None
        self.rear_cart_attached = False
        self.scenario_recovery_active = False
        self.detach_release_retry_count = 0
        self.scenario2_front_attach_retry_count = 0
        self.rejoin_mode = None
        self.update_dynamic_state("scenario_complete")
        self.enable_joystick_control()
        self.set_state(State.IDLE, reason)

    def abort_scenario(self, reason, return_joystick=True):
        if self.recover_scenario_failure(reason):
            return

        self.get_logger().error(f"scenario aborted: {reason}")
        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.publish_dock_prep_done(False)
        self.publish_rl_docking_ready(False)
        self.clear_rl_docking_sequence_state()
        self.publish_docking_target_burst(0, "abort_target_reset")
        self.clear_patrol_resume_after_docking()
        self.active_scenario_id = 0
        self.detach_pose = None
        self.set_rear_cart_attached(False, "abort")
        self.scenario_recovery_active = False
        self.detach_release_retry_count = 0
        self.scenario2_front_attach_retry_count = 0
        self.rejoin_mode = None
        if return_joystick:
            self.enable_joystick_control()
        self.set_state(State.IDLE, "abort")

    def set_state(self, next_state: State, reason: str):
        if self.state != next_state:
            self.get_logger().info(
                f"state {self.state.value} -> {next_state.value} ({reason})"
            )
            self.state = next_state

    # ------------------------------------------------------------------
    # Timers and commands
    # ------------------------------------------------------------------
    def schedule_once(self, key, delay_sec, callback):
        self.cancel_timer(key)
        delay_sec = max(0.0, float(delay_sec))
        if delay_sec <= 0.0:
            callback()
            return

        def run_once():
            self.cancel_timer(key)
            callback()

        self.scenario_timers[key] = self.create_timer(delay_sec, run_once)

    def cancel_timer(self, key):
        timer = self.scenario_timers.pop(key, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def cancel_all_timers(self):
        for key in list(self.scenario_timers):
            self.cancel_timer(key)

    def publish_bool_burst(self, key, publisher, value):
        self.cancel_timer(key)
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

        if self.command_burst_duration_sec <= 0.0:
            return

        start_time = self.get_clock().now()

        def republish():
            publisher.publish(msg)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed >= self.command_burst_duration_sec:
                self.cancel_timer(key)

        self.scenario_timers[key] = self.create_timer(
            self.command_burst_period_sec,
            republish,
        )

    def publish_int_burst(self, key, publisher, value):
        self.cancel_timer(key)
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

        if self.command_burst_duration_sec <= 0.0:
            return

        start_time = self.get_clock().now()

        def republish():
            publisher.publish(msg)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed >= self.command_burst_duration_sec:
                self.cancel_timer(key)

        self.scenario_timers[key] = self.create_timer(
            self.command_burst_period_sec,
            republish,
        )

    def publish_bool_once(self, publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def enable_navigation_control(self):
        self.cancel_timer("navigation_control_hold")
        self.cancel_timer("rear_nav_control")
        self.cancel_timer("front_nav_control")
        self.publish_bool_once(self.rear_joy_sig_pub, False)
        self.publish_bool_once(self.front_joy_sig_pub, False)

    def enable_joystick_control(self):
        self.cancel_timer("navigation_control_hold")
        self.publish_bool_burst("rear_joy_control", self.rear_joy_sig_pub, True)
        self.publish_bool_burst("front_joy_control", self.front_joy_sig_pub, True)

    # ------------------------------------------------------------------
    # Goal cancel and route helpers
    # ------------------------------------------------------------------
    def cancel_rear_goal(self):
        self.active_rear_goal_seq += 1
        self.active_rear_goal_pending = False
        self.active_rear_goal_label = None
        if self.active_rear_goal_handle is not None:
            self.active_rear_goal_handle.cancel_goal_async()
            self.active_rear_goal_handle = None

    def cancel_front_goal(self):
        previous_proxy_goal_id = self.active_front_proxy_goal_id
        self.cancel_timer("front_goal_timeout")
        self.active_front_goal_seq += 1
        self.active_front_goal_pending = False
        self.active_front_proxy_goal_id = 0
        self.active_front_goal_label = None
        if self.front_nav_transport == "topic_proxy" and previous_proxy_goal_id:
            msg = String()
            msg.data = json.dumps({"id": previous_proxy_goal_id, "cancel": True})
            self.front_nav_cancel_pub.publish(msg)
        if self.active_front_goal_handle is not None:
            self.active_front_goal_handle.cancel_goal_async()
            self.active_front_goal_handle = None

    def publish_dock_prep_done(self, done: bool):
        if done:
            self.publish_bool_burst(
                "dock_prep_done",
                self.dock_prep_done_pub,
                True,
            )
            return

        self.cancel_timer("dock_prep_done")
        msg = Bool()
        msg.data = False
        self.dock_prep_done_pub.publish(msg)

    def publish_rl_docking_ready(self, ready: bool):
        if ready:
            self.publish_bool_burst(
                "rl_docking_ready",
                self.rl_docking_ready_pub,
                True,
            )
            return

        self.cancel_timer("rl_docking_ready")
        msg = Bool()
        msg.data = False
        self.rl_docking_ready_pub.publish(msg)

    def publish_rear_docking_target(self, target: int):
        msg = Int32()
        msg.data = int(target)
        self.docking_target_pub.publish(msg)

    def publish_front_docking_target(self, target: int):
        msg = Int32()
        msg.data = int(target)
        self.front_docking_target_pub.publish(msg)

    def publish_docking_target(self, target: int):
        self.publish_rear_docking_target(target)
        self.publish_front_docking_target(target)

    def publish_docking_target_burst(self, target: int, key_prefix="docking_target"):
        self.publish_int_burst(
            f"{key_prefix}_rear",
            self.docking_target_pub,
            target,
        )
        self.publish_int_burst(
            f"{key_prefix}_front",
            self.front_docking_target_pub,
            target,
        )

    def publish_cart_count(self, count: int):
        msg = UInt16()
        msg.data = max(0, int(count))
        self.cart_count_pub.publish(msg)

    def prepare_cart_mode_route_goal_costmaps(self):
        if not self.clear_rear_costmaps_on_cart_mode:
            return False
        if not (self.is_attached and self.cart_count >= 1):
            return False
        if self.active_route_type != "PATROL":
            return False

        now = self.get_clock().now()
        if self.last_rear_costmap_clear_time is not None:
            elapsed = (
                now - self.last_rear_costmap_clear_time
            ).nanoseconds * 1e-9
            if elapsed < self.rear_costmap_clear_min_interval_sec:
                return False

        if not self.clear_rear_costmaps(
            "cart mode route goal %d/%d"
            % (
                self.active_route_index + 1,
                len(self.active_route_poses),
            )
        ):
            return False

        self.last_rear_costmap_clear_time = now
        return self.rear_costmap_clear_settle_sec > 0.0

    def clear_rear_costmaps(self, reason: str):
        sent = 0
        request = ClearEntireCostmap.Request()
        for label, client, service_name in (
            (
                "global",
                self.rear_global_costmap_clear_client,
                self.rear_global_costmap_clear_service,
            ),
            (
                "local",
                self.rear_local_costmap_clear_client,
                self.rear_local_costmap_clear_service,
            ),
        ):
            if not client.wait_for_service(
                timeout_sec=self.rear_costmap_clear_service_timeout_sec
            ):
                self.get_logger().warn(
                    f"rear {label} costmap clear service is not ready: {service_name}"
                )
                continue
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, name=label: self.clear_costmap_callback(fut, name)
            )
            sent += 1

        if sent > 0:
            self.get_logger().info(
                f"rear Nav2 costmap clear requested ({reason})"
            )
        return sent > 0

    def clear_costmap_callback(self, future: Future, costmap_name: str):
        try:
            future.result()
        except Exception as ex:
            self.get_logger().warn(
                f"rear {costmap_name} costmap clear failed: {ex}"
            )

    def pause_rear_nav2(self, reason: str):
        self.call_rear_nav2_lifecycle(
            ManageLifecycleNodes.Request.PAUSE,
            "pause",
            reason,
        )

    def resume_rear_nav2(self, reason: str):
        self.call_rear_nav2_lifecycle(
            ManageLifecycleNodes.Request.RESUME,
            "resume",
            reason,
        )

    def call_rear_nav2_lifecycle(self, command: int, action: str, reason: str):
        if not self.manage_rear_nav2_lifecycle:
            return

        if action == "pause" and self.rear_nav2_paused:
            return

        if not self.rear_nav2_lifecycle_client.wait_for_service(
            timeout_sec=self.rear_nav2_lifecycle_service_timeout_sec
        ):
            self.get_logger().warn(
                "rear Nav2 lifecycle service is not ready: "
                f"{self.rear_nav2_lifecycle_service}"
            )
            return

        request = ManageLifecycleNodes.Request()
        request.command = command
        future = self.rear_nav2_lifecycle_client.call_async(request)
        future.add_done_callback(
            lambda fut, act=action, why=reason: self.rear_nav2_lifecycle_callback(
                fut,
                act,
                why,
            )
        )
        self.get_logger().info(f"rear Nav2 lifecycle {action} requested ({reason})")

    def rear_nav2_lifecycle_callback(self, future: Future, action: str, reason: str):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"rear Nav2 lifecycle {action} failed: {ex}")
            return

        if not response.success:
            self.get_logger().warn(
                f"rear Nav2 lifecycle {action} rejected ({reason})"
            )
            return

        self.rear_nav2_paused = action == "pause"
        self.get_logger().info(
            f"rear Nav2 lifecycle {action} complete ({reason})"
        )

    def clear_route(self):
        self.cancel_timer("route_goal_retry")
        self.cancel_timer("route_tf_check")
        self.cancel_timer("route_goal_costmap_settle")
        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0
        self.active_route_patrol_start_index = 0
        self.route_goal_retry_count = 0

    def clean_points(self, points):
        clean = []
        for point in points:
            xy = self.waypoint_xy(point)
            if xy is None:
                continue
            x, y = xy
            yaw = self.waypoint_explicit_yaw(point)
            if yaw is None:
                clean.append((x, y))
            else:
                clean.append((x, y, yaw))
        return clean

    def create_path_poses(self, points, final_yaw=None):
        poses = []
        for index, point in enumerate(points):
            xy = self.waypoint_xy(point)
            if xy is None:
                continue
            x, y = xy
            explicit_yaw = self.waypoint_explicit_yaw(point)
            if explicit_yaw is not None:
                yaw = explicit_yaw
            elif index < len(points) - 1:
                next_xy = self.waypoint_xy(points[index + 1])
                if next_xy is None:
                    yaw = final_yaw if final_yaw is not None else 0.0
                else:
                    next_x, next_y = next_xy
                    yaw = math.atan2(next_y - y, next_x - x)
            elif final_yaw is not None:
                yaw = final_yaw
            elif index > 0:
                prev_xy = self.waypoint_xy(points[index - 1])
                if prev_xy is None:
                    yaw = 0.0
                else:
                    prev_x, prev_y = prev_xy
                    yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0
            poses.append(self.create_pose_stamped(x, y, yaw))
        return poses

    def route_goal_reached_by_tf(self):
        if self.active_route_index >= len(self.active_route_poses):
            return True

        robot_pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if robot_pose is None:
            self.get_logger().warn("route success accepted without TF verification.")
            return True

        goal = self.active_route_poses[self.active_route_index]
        distance = math.hypot(
            goal.pose.position.x - robot_pose[0],
            goal.pose.position.y - robot_pose[1],
        )
        if distance <= self.route_arrival_tolerance:
            return True

        self.get_logger().warn(
            "route goal result succeeded but TF distance is %.2fm (tol %.2fm)"
            % (distance, self.route_arrival_tolerance)
        )
        return False

    def schedule_route_tf_check(self):
        if self.route_tf_check_period_sec <= 0.0:
            return
        if self.active_route_type not in ("PATROL", "EXIT"):
            return
        self.schedule_once(
            "route_tf_check",
            self.route_tf_check_period_sec,
            self.check_route_tf_fallback,
        )

    def check_route_tf_fallback(self):
        if self.active_route_type not in ("PATROL", "EXIT"):
            return
        if self.accept_current_route_goal_by_tf("periodic TF check"):
            return
        self.schedule_route_tf_check()

    def accept_current_route_goal_by_tf(self, reason: str, cancel_active=True):
        if self.active_route_type not in ("PATROL", "EXIT"):
            return False
        if self.active_route_index >= len(self.active_route_poses):
            return False

        robot_pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if robot_pose is None:
            return False

        route_type = self.active_route_type
        route_index = self.active_route_index
        route_count = len(self.active_route_poses)
        goal = self.active_route_poses[route_index]
        distance = math.hypot(
            goal.pose.position.x - robot_pose[0],
            goal.pose.position.y - robot_pose[1],
        )
        if distance > self.route_arrival_tolerance:
            return False

        if cancel_active:
            self.cancel_rear_goal()
        self.cancel_timer("route_tf_check")
        self.cancel_timer("route_goal_retry")

        if route_type == "PATROL":
            completed_patrol_index = (
                self.active_route_patrol_start_index
                + route_index
                + 1
            )
            self.current_patrol_waypoint_index = max(
                self.current_patrol_waypoint_index,
                completed_patrol_index,
            )

        self.route_goal_retry_count = 0
        self.active_route_index = route_index + 1
        self.get_logger().info(
            "route %s goal %d/%d accepted by TF fallback (%s): dist=%.2fm/%.2fm"
            % (
                route_type,
                route_index + 1,
                route_count,
                reason,
                distance,
                self.route_arrival_tolerance,
            )
        )
        self.send_current_route_goal()
        return True

    # ------------------------------------------------------------------
    # Geometry and path math
    # ------------------------------------------------------------------
    def point_stamped_to_map_pose(self, msg: PointStamped):
        if not (math.isfinite(msg.point.x) and math.isfinite(msg.point.y)):
            self.get_logger().warn("cart target has invalid values.")
            return None

        source_frame = self.normalize_frame_id(msg.header.frame_id)
        local_pose = PoseStamped()
        local_pose.header.frame_id = source_frame
        local_pose.header.stamp = self.get_clock().now().to_msg()
        local_pose.pose.position.x = msg.point.x
        local_pose.pose.position.y = msg.point.y
        local_pose.pose.orientation.w = 1.0

        try:
            if source_frame and source_frame != "map":
                transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
                local_pose.pose = tf2_geometry_msgs.do_transform_pose(
                    local_pose.pose,
                    transform,
                )
            local_pose.header.frame_id = "map"
            return local_pose
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"cart target TF failed: {ex}")
            return None

    def pose_to_map_waypoint(self, pose, source_frame, label):
        if not (math.isfinite(pose.position.x) and math.isfinite(pose.position.y)):
            self.get_logger().warn(f"{label} has invalid x/y.")
            return None

        source_frame = self.normalize_frame_id(source_frame) or "map"
        if source_frame == "map":
            yaw = self.pose_orientation_yaw_or_none(pose)
            if yaw is None:
                return (float(pose.position.x), float(pose.position.y))
            return (float(pose.position.x), float(pose.position.y), yaw)

        try:
            transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
            map_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"{label} TF failed: {ex}")
            return None

        yaw = self.pose_orientation_yaw_or_none(map_pose)
        if yaw is None:
            return (float(map_pose.position.x), float(map_pose.position.y))
        return (float(map_pose.position.x), float(map_pose.position.y), yaw)

    def pose_to_map_xy(self, pose, source_frame, label):
        waypoint = self.pose_to_map_waypoint(pose, source_frame, label)
        if waypoint is None:
            return None
        return self.waypoint_xy(waypoint)

    def waypoint_xy(self, waypoint):
        if isinstance(waypoint, dict):
            position = self.waypoint_position_dict(waypoint)
            if position is None:
                return None
            try:
                x = float(position.get("x"))
                y = float(position.get("y"))
            except (TypeError, ValueError):
                return None

            if not (math.isfinite(x) and math.isfinite(y)):
                return None
            return (x, y)

        try:
            if len(waypoint) < 2:
                return None
            x = float(waypoint[0])
            y = float(waypoint[1])
        except (TypeError, ValueError):
            return None

        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return (x, y)

    def waypoint_explicit_yaw(self, waypoint):
        if isinstance(waypoint, dict):
            if "yaw" in waypoint:
                try:
                    yaw = float(waypoint["yaw"])
                except (TypeError, ValueError):
                    return None
                if math.isfinite(yaw):
                    return self.normalize_angle(yaw)

            orientation = self.waypoint_orientation_dict(waypoint)
            if orientation is None:
                return None
            return self.quaternion_yaw_or_none(
                orientation.get("x", 0.0),
                orientation.get("y", 0.0),
                orientation.get("z", 0.0),
                orientation.get("w", 1.0),
            )

        try:
            if len(waypoint) < 3:
                return None
        except (TypeError, ValueError):
            return None

        try:
            if len(waypoint) == 3:
                yaw = float(waypoint[2])
                if math.isfinite(yaw):
                    return self.normalize_angle(yaw)
                return None

            if len(waypoint) == 4:
                return self.quaternion_yaw_or_none(0.0, 0.0, waypoint[2], waypoint[3])

            if len(waypoint) == 6:
                return self.quaternion_yaw_or_none(
                    waypoint[2],
                    waypoint[3],
                    waypoint[4],
                    waypoint[5],
                )

            if len(waypoint) >= 7:
                return self.quaternion_yaw_or_none(
                    waypoint[3],
                    waypoint[4],
                    waypoint[5],
                    waypoint[6],
                )
        except (TypeError, ValueError):
            return None

        return None

    def waypoint_position_dict(self, waypoint):
        pose = waypoint.get("pose", waypoint)
        return pose.get("position") if isinstance(pose, dict) else None

    def waypoint_orientation_dict(self, waypoint):
        pose = waypoint.get("pose", waypoint)
        return pose.get("orientation") if isinstance(pose, dict) else None

    def pose_orientation_yaw_or_none(self, pose):
        q = pose.orientation
        return self.quaternion_yaw_or_none(q.x, q.y, q.z, q.w)

    def quaternion_yaw_or_none(self, qx, qy, qz, qw):
        try:
            qx = float(qx)
            qy = float(qy)
            qz = float(qz)
            qw = float(qw)
        except (TypeError, ValueError):
            return None

        if not all(math.isfinite(value) for value in (qx, qy, qz, qw)):
            return None

        norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
        if norm_sq <= 1e-9:
            return None

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return self.normalize_angle(math.atan2(siny_cosp, cosy_cosp))

    def robot_pose_in_map(self, frame_candidates):
        for frame_id in frame_candidates:
            frame_id = self.normalize_frame_id(frame_id)
            try:
                transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
                translation = transform.transform.translation
                yaw = self.yaw_from_quaternion(transform.transform.rotation)
                return (translation.x, translation.y, yaw)
            except tf2_ros.TransformException:
                continue
        return None

    def get_robot_xy_in_map(self):
        pose = self.robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if pose is None:
            return None
        return (pose[0], pose[1])

    def current_progress_on_path(self):
        fallback = self.waypoint_progress(self.current_patrol_waypoint_index - 1)
        robot_xy = self.get_robot_xy_in_map()
        if robot_xy is None:
            return fallback
        projection = self.closest_point_on_path(
            robot_xy[0],
            robot_xy[1],
            min_progress=fallback,
        )
        return projection["progress"]

    def closest_point_on_path(self, target_x, target_y, min_progress=0.0):
        cumulative = self.path_cumulative_lengths()
        best = None
        best_dist = float("inf")

        for index in range(len(self.patrol_path) - 1):
            start_xy = self.waypoint_xy(self.patrol_path[index])
            end_xy = self.waypoint_xy(self.patrol_path[index + 1])
            if start_xy is None or end_xy is None:
                continue
            ax, ay = start_xy
            bx, by = end_xy
            ab_x = bx - ax
            ab_y = by - ay
            ab_len_sq = ab_x * ab_x + ab_y * ab_y
            if ab_len_sq <= 1e-9:
                continue

            t = ((target_x - ax) * ab_x + (target_y - ay) * ab_y) / ab_len_sq
            t = max(0.0, min(1.0, t))
            ab_len = math.sqrt(ab_len_sq)
            progress = cumulative[index] + t * ab_len
            if progress + 1e-6 < min_progress:
                continue

            x = ax + t * ab_x
            y = ay + t * ab_y
            distance = math.hypot(target_x - x, target_y - y)
            if distance < best_dist:
                best_dist = distance
                best = {
                    "point": (x, y),
                    "segment_index": index,
                    "t": t,
                    "progress": progress,
                    "distance": distance,
                    "path_yaw": math.atan2(ab_y, ab_x),
                }

        if best is not None:
            return best

        last_x, last_y = self.waypoint_xy(self.patrol_path[-1])
        prev_x, prev_y = self.waypoint_xy(self.patrol_path[-2])
        return {
            "point": (last_x, last_y),
            "segment_index": max(0, len(self.patrol_path) - 2),
            "t": 1.0,
            "progress": cumulative[-1],
            "distance": math.hypot(target_x - last_x, target_y - last_y),
            "path_yaw": math.atan2(last_y - prev_y, last_x - prev_x),
        }

    def build_route_to_exit(self, exit_projection, current_progress):
        exit_progress = exit_projection["progress"]
        exit_point = exit_projection["point"]
        if self.cart_exit_direct_route:
            return [exit_point]

        cumulative = self.path_cumulative_lengths()
        route = []

        for index, waypoint in enumerate(self.patrol_path):
            waypoint_progress = cumulative[index]
            if current_progress + 0.05 < waypoint_progress < exit_progress - 0.05:
                route.append(waypoint)

        last_route_xy = self.waypoint_xy(route[-1]) if route else None
        if (
            last_route_xy is None
            or math.hypot(last_route_xy[0] - exit_point[0], last_route_xy[1] - exit_point[1])
            > 0.05
        ):
            route.append(exit_point)

        return route

    def path_cumulative_lengths(self):
        cumulative = [0.0]
        for index in range(len(self.patrol_path) - 1):
            start_xy = self.waypoint_xy(self.patrol_path[index])
            end_xy = self.waypoint_xy(self.patrol_path[index + 1])
            if start_xy is None or end_xy is None:
                cumulative.append(cumulative[-1])
                continue
            ax, ay = start_xy
            bx, by = end_xy
            cumulative.append(cumulative[-1] + math.hypot(bx - ax, by - ay))
        return cumulative

    def waypoint_progress(self, waypoint_index):
        if waypoint_index < 0:
            return 0.0
        cumulative = self.path_cumulative_lengths()
        return cumulative[min(waypoint_index, len(cumulative) - 1)]

    def next_patrol_waypoint_index_after_progress(self, progress: float):
        cumulative = self.path_cumulative_lengths()
        for index, waypoint_progress in enumerate(cumulative):
            if waypoint_progress > progress + 0.05:
                return index
        return len(cumulative)

    def patrol_waypoint_yaw(self, waypoint_index: int):
        if len(self.patrol_path) < 2:
            return 0.0

        waypoint_index = max(0, min(int(waypoint_index), len(self.patrol_path) - 1))
        explicit_yaw = self.waypoint_explicit_yaw(self.patrol_path[waypoint_index])
        if explicit_yaw is not None:
            return explicit_yaw

        if waypoint_index < len(self.patrol_path) - 1:
            x0, y0 = self.waypoint_xy(self.patrol_path[waypoint_index])
            x1, y1 = self.waypoint_xy(self.patrol_path[waypoint_index + 1])
            return math.atan2(y1 - y0, x1 - x0)

        x0, y0 = self.waypoint_xy(self.patrol_path[waypoint_index - 1])
        x1, y1 = self.waypoint_xy(self.patrol_path[waypoint_index])
        return math.atan2(y1 - y0, x1 - x0)

    def pose2d_to_map_pose(self, msg: Pose2D):
        frame_id = self.precise_pose2d_frame
        cart_yaw_in_frame = self.normalize_angle(-float(msg.theta))
        pose_stamped = self.create_pose_stamped(
            msg.x,
            msg.y,
            cart_yaw_in_frame,
            frame_id,
        )
        if not frame_id or frame_id == "map":
            return pose_stamped.pose

        transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
        return tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)

    def target_point_to_map_pose(self, msg: PointStamped):
        frame_id = self.precise_pose2d_frame
        cart_yaw_in_frame = self.normalize_angle(-float(msg.point.z))
        pose_stamped = self.create_pose_stamped(
            msg.point.x,
            msg.point.y,
            cart_yaw_in_frame,
            frame_id,
        )
        if not frame_id or frame_id == "map":
            return pose_stamped.pose

        transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
        return tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)

    def parse_aruco_id(self, frame_id):
        value = str(frame_id or "").strip()
        if not value:
            return None

        token = value.rsplit("/", 1)[-1]
        if token.startswith("aruco_"):
            token = token[len("aruco_") :]

        try:
            return int(token)
        except ValueError:
            return None

    def cart_center_from_aruco_marker(self, marker_x, marker_y, cart_yaw, aruco_id):
        front_back = self.cart_marker_front_back_offset_m
        left_right = self.cart_marker_left_right_offset_m
        marker_offsets = {
            0: (front_back, 0.0),
            1: (-front_back, 0.0),
            2: (0.0, left_right),
            3: (0.0, -left_right),
        }

        if aruco_id not in marker_offsets:
            return marker_x, marker_y, False

        marker_dx, marker_dy = marker_offsets[aruco_id]
        offset_x = marker_dx * math.cos(cart_yaw) - marker_dy * math.sin(cart_yaw)
        offset_y = marker_dx * math.sin(cart_yaw) + marker_dy * math.cos(cart_yaw)
        return marker_x - offset_x, marker_y - offset_y, True

    def consume_pending_precise_pose(self):
        msg = self.pending_precise_pose_msg
        msg_type = self.pending_precise_pose_type
        self.pending_precise_pose_msg = None
        self.pending_precise_pose_type = None

        if msg is None:
            return

        self.get_logger().info("processing pending precise pose.")
        if msg_type == "PoseStamped":
            self.precise_pose_callback(msg)
        elif msg_type == "Pose2D":
            self.precise_pose2d_callback(msg)
        elif msg_type == "PointStamped":
            self.precise_target_point_callback(msg)

    def clear_precise_pose(self):
        self.pending_precise_pose_msg = None
        self.pending_precise_pose_type = None
        self.last_dock_prep_rear_goal = None
        self.cancel_timer("precise_pose_timeout")

    def cart_stop_distance(self):
        if self.is_attached:
            return self.param_float("attached_cart_stop_distance", 1.5)
        return self.param_float("detached_cart_stop_distance", 1.0)

    # ------------------------------------------------------------------
    # Generic ROS helpers
    # ------------------------------------------------------------------
    def send_parameters_to_node(self, node_name: str, param_dict: dict):
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")
        if not client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn(f"{node_name} set_parameters service not ready.")
            return

        request = SetParameters.Request()
        for name, value in param_dict.items():
            try:
                request.parameters.append(
                    Parameter(name, value=value).to_parameter_msg()
                )
            except Exception as ex:
                self.get_logger().error(f"parameter conversion failed: {name}: {ex}")

        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, node=node_name: self.parameter_set_callback(fut, node)
        )

    def parameter_set_callback(self, future: Future, node_name: str):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"{node_name} parameter call failed: {ex}")
            return

        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            self.get_logger().warn(f"{node_name} rejected parameters: {failed}")

    def current_behavior_tree(self):
        if self.rear_cart_attached:
            return self.rear_cart_diff_bt_path
        if not self.is_attached:
            return self.diff_bt_path
        if self.cart_count >= 1:
            return self.ackermann_cart_bt_path
        return self.ackermann_bt_path

    def create_polygon(self, front_x: float, rear_x: float, width: float):
        poly = Polygon()
        poly.points = [
            Point32(x=float(front_x), y=float(-width), z=0.0),
            Point32(x=float(front_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(-width), z=0.0),
        ]
        return poly

    def create_pose_stamped(self, x, y, yaw, frame_id="map"):
        pose = PoseStamped()
        pose.header.frame_id = self.normalize_frame_id(frame_id) or "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def pose_xy_is_finite(self, pose_stamped: PoseStamped):
        return (
            math.isfinite(pose_stamped.pose.position.x)
            and math.isfinite(pose_stamped.pose.position.y)
        )

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def normalize_frame_id(self, frame_id):
        return str(frame_id or "").strip().lstrip("/")

    def param_float(self, name, default):
        return max(0.0, float(self.get_parameter(name).value if self.has_parameter(name) else default))

    def param_int(self, name, default):
        return max(0, int(self.get_parameter(name).value if self.has_parameter(name) else default))


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
