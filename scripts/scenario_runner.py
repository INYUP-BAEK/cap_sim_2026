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
    PoseStamped,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String, UInt16

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
    WAIT_ATTACH = "WAIT_ATTACH"


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
                ("footprint_publish_period_sec", 0.5),
                ("nav_server_timeout_sec", 2.0),
                ("route_goal_max_retries", 2),
                ("route_goal_retry_delay_sec", 1.0),
                ("state_max_retries", 8),
                ("tf_retry_delay_sec", 0.5),
                ("route_arrival_tolerance", 0.35),
                ("front_clear_after_detach_delay_sec", 2.0),
                ("front_clear_distance", 0.5),
                ("detach_timeout_sec", 12.0),
                ("precise_pose_timeout_sec", 12.0),
                ("attach_timeout_sec", 60.0),
                ("front_goal_timeout_sec", 30.0),
                ("dock_goal_offset", 1.0),
                ("precise_pose2d_frame", "base_link"),
                ("command_burst_duration_sec", 0.5),
                ("command_burst_period_sec", 0.1),
                ("release_to_joystick_on_wait_attach", True),
                ("auto_attach_after_dock_prep", False),
                ("front_nav_transport", "topic_proxy"),
                ("front_nav_goal_topic", "/front/scenario_nav_goal"),
                ("front_nav_cancel_topic", "/front/scenario_nav_cancel"),
                ("front_nav_result_topic", "/front/scenario_nav_result"),
                ("dock_prep_done_topic", "/dock_prep_done"),
                ("manage_rear_nav2_lifecycle", True),
                ("pause_rear_nav2_on_dock_prep_done", True),
                ("resume_rear_nav2_on_scenario_start", True),
                (
                    "rear_nav2_lifecycle_service",
                    "/lifecycle_manager_navigation/manage_nodes",
                ),
                ("rear_nav2_lifecycle_service_timeout_sec", 2.0),
                ("rear_nav2_resume_delay_sec", 1.0),
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
        self.nav_server_timeout_sec = self.param_float("nav_server_timeout_sec", 2.0)
        self.route_goal_max_retries = self.param_int("route_goal_max_retries", 2)
        self.route_goal_retry_delay_sec = self.param_float(
            "route_goal_retry_delay_sec",
            1.0,
        )
        self.state_max_retries = self.param_int("state_max_retries", 8)
        self.tf_retry_delay_sec = self.param_float("tf_retry_delay_sec", 0.5)
        self.route_arrival_tolerance = self.param_float(
            "route_arrival_tolerance",
            0.35,
        )
        self.front_clear_after_detach_delay_sec = self.param_float(
            "front_clear_after_detach_delay_sec",
            2.0,
        )
        self.front_clear_distance = self.param_float("front_clear_distance", 0.5)
        self.detach_timeout_sec = self.param_float("detach_timeout_sec", 12.0)
        self.precise_pose_timeout_sec = self.param_float(
            "precise_pose_timeout_sec",
            12.0,
        )
        self.attach_timeout_sec = self.param_float("attach_timeout_sec", 60.0)
        self.front_goal_timeout_sec = self.param_float("front_goal_timeout_sec", 30.0)
        self.dock_goal_offset = self.param_float("dock_goal_offset", 1.0)
        self.precise_pose2d_frame = self.normalize_frame_id(
            self.get_parameter("precise_pose2d_frame").value
        )
        self.command_burst_duration_sec = self.param_float(
            "command_burst_duration_sec",
            0.5,
        )
        self.command_burst_period_sec = max(
            0.02,
            self.param_float("command_burst_period_sec", 0.1),
        )
        self.release_to_joystick_on_wait_attach = bool(
            self.get_parameter("release_to_joystick_on_wait_attach").value
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

        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
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

        self.active_rear_goal_handle = None
        self.active_rear_goal_seq = 0
        self.active_rear_goal_pending = False
        self.active_rear_goal_label = None

        self.active_front_goal_handle = None
        self.active_front_goal_seq = 0
        self.active_front_goal_pending = False
        self.active_front_goal_label = None
        self.active_front_proxy_goal_id = 0

        self.patrol_path = [(6.15, -0.52), (7.02, 1.95)]
        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0
        self.route_goal_retry_count = 0
        self.current_patrol_waypoint_index = 0
        self.rear_nav2_paused = False

        self.detected_cart_pose = None
        self.pending_precise_pose_msg = None
        self.pending_precise_pose_type = None
        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.last_cart_goal_time = None
        self.front_clear_tf_retries = 0
        self.front_clear_goal_retry_count = 0
        self.rear_align_tf_retries = 0
        self.rear_align_goal_retry_count = 0
        self.precise_goal_retries = 0
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
        self.rear_joy_sig_pub = self.create_publisher(Bool, "/joy_control_sig", 10)
        self.front_joy_sig_pub = self.create_publisher(
            Bool,
            "/front/joy_control_sig",
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.topic_subs = [
            self.create_subscription(Bool, "/docking_state", self.docking_callback, 10),
            self.create_subscription(UInt16, "/cart_count", self.cart_count_callback, 10),
            self.create_subscription(
                PoseStamped,
                "/mission_goal",
                self.mission_goal_callback,
                10,
            ),
            self.create_subscription(
                PointStamped,
                "/vision/cart_target_ground",
                self.cart_target_callback,
                10,
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
                Pose2D,
                "/rear/rs/cart_pose",
                self.precise_pose2d_callback,
                10,
            ),
            self.create_subscription(
                String,
                self.front_nav_result_topic,
                self.front_nav_proxy_result_callback,
                10,
            ),
            self.create_subscription(
                Empty,
                "/start_patrol_mission",
                self.start_mission_callback,
                10,
            ),
            self.create_subscription(Path, "/mission_path", self.mission_path_callback, 10),
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
        if self.is_attached != next_attached:
            self.is_attached = next_attached
            self.get_logger().info(f"docking_state -> attached={self.is_attached}")
            self.update_dynamic_state("docking_state")
        else:
            self.publish_footprints()

        self.handle_docking_transition(next_attached)

    def cart_count_callback(self, msg: UInt16):
        if self.cart_count == msg.data:
            self.publish_footprints()
            return

        self.cart_count = int(msg.data)
        self.get_logger().info(f"cart_count -> {self.cart_count}")
        self.update_dynamic_state("cart_count")

    def update_dynamic_state(self, reason: str):
        self.publish_footprints()
        smoother_params, mode = self.current_velocity_params()
        self.send_parameters_to_node("/velocity_smoother", smoother_params)
        self.get_logger().info(f"dynamic update ({reason}): {mode}")

    def current_velocity_params(self):
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
                "max_velocity": [0.35, 0.0, 1.0],
                "min_velocity": [-0.35, 0.0, -1.0],
                "max_accel": [0.5, 0.0, 1.5],
                "max_decel": [-0.5, 0.0, -1.5],
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
    def start_mission_callback(self, msg: Empty):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return
        self.start_patrol_route(self.patrol_path, "start_patrol_mission")

    def mission_path_callback(self, msg: Path):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return

        frame_id = self.normalize_frame_id(msg.header.frame_id) or "map"
        points = []
        for index, pose_stamped in enumerate(msg.poses):
            source_frame = self.normalize_frame_id(pose_stamped.header.frame_id) or frame_id
            point = self.pose_to_map_xy(pose_stamped.pose, source_frame, f"path[{index}]")
            if point is not None:
                points.append(point)

        self.start_patrol_route(points, "mission_path")

    def mission_waypoints_callback(self, msg: PoseArray):
        if not self.enable_auto_scenario:
            self.get_logger().warn("auto scenario is disabled.")
            return

        source_frame = self.normalize_frame_id(msg.header.frame_id) or "map"
        points = []
        for index, pose in enumerate(msg.poses):
            point = self.pose_to_map_xy(pose, source_frame, f"waypoint[{index}]")
            if point is not None:
                points.append(point)

        self.start_patrol_route(points, "mission_waypoints")

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

    def start_patrol_route(self, points, source: str):
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

        self.enable_navigation_control()
        wait_for_resume = self.resume_rear_nav2_on_scenario_start and self.rear_nav2_paused
        if self.resume_rear_nav2_on_scenario_start:
            self.resume_rear_nav2(source)
        self.set_state(State.PATROL, source)
        if wait_for_resume:
            self.schedule_once(
                "rear_nav2_resume_start_route",
                self.rear_nav2_resume_delay_sec,
                lambda route=clean_points: self.start_route(route, "PATROL")
                if self.state == State.PATROL
                else None,
            )
            return True
        return self.start_route(clean_points, "PATROL")

    # ------------------------------------------------------------------
    # Cart detection and route-to-exit
    # ------------------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        if self.state != State.PATROL:
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
            "cart detected at map=(%.2f, %.2f). exit=(%.2f, %.2f)"
            % (
                cart_pose.pose.position.x,
                cart_pose.pose.position.y,
                exit_projection["point"][0],
                exit_projection["point"][1],
            )
        )

        self.cancel_rear_goal()
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
        self.front_clear_tf_retries = 0
        self.front_clear_goal_retry_count = 0
        self.rear_align_tf_retries = 0
        self.rear_align_goal_retry_count = 0
        self.precise_goal_retries = 0

        if not self.is_attached:
            self.get_logger().info("already detached; skipping detach wait.")
            self.schedule_front_clear()
            return

        self.set_state(State.WAIT_DETACH, "detach_request")
        self.enable_navigation_control()
        self.publish_bool_burst("gripper_release", self.gripper_toggle_pub, False)
        self.publish_bool_burst("front_home", self.front_home_pub, True)

        if self.detach_timeout_sec > 0.0:
            self.schedule_once(
                "detach_timeout",
                self.detach_timeout_sec,
                self.handle_detach_timeout,
            )

        self.get_logger().info(
            "detach requested. Waiting for /docking_state=false from controller or joystick."
        )

    def handle_docking_transition(self, attached: bool):
        if self.state == State.WAIT_DETACH and not attached:
            self.cancel_timer("detach_timeout")
            self.schedule_front_clear()
            return

        if self.state == State.WAIT_ATTACH and attached:
            self.cancel_timer("attach_timeout")
            self.complete_scenario("attached")
            return

        if attached and self.state in (
            State.FRONT_CLEAR,
            State.REAR_ALIGN,
            State.WAIT_PRECISE_POSE,
            State.DOCK_PREP,
        ):
            self.get_logger().warn(
                f"docking_state=true during {self.state.value}; finishing scenario early."
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

        front_pose = self.robot_pose_in_map(("front/base_footprint", "front/base_link"))
        if front_pose is None:
            self.front_clear_tf_retries += 1
            if self.front_clear_tf_retries > self.state_max_retries:
                self.abort_scenario("front TF unavailable for clear move")
                return
            self.get_logger().warn(
                "front TF unavailable. retry %d/%d"
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
        goal = self.create_pose_stamped(
            x + self.front_clear_distance * math.cos(yaw),
            y + self.front_clear_distance * math.sin(yaw),
            yaw,
        )

        self.get_logger().info(
            f"sending front clear goal: {self.front_clear_distance:.2f}m forward."
        )
        if not self.send_front_goal(goal, "FRONT_CLEAR"):
            self.retry_or_abort("front_clear_goal_retry", self.start_front_clear_move)

    def start_rear_heading_alignment(self):
        if self.state != State.REAR_ALIGN:
            self.set_state(State.REAR_ALIGN, "front_clear_done")

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
        rear_x, rear_y, _ = rear_pose
        cart_x = self.detected_cart_pose.pose.position.x
        cart_y = self.detected_cart_pose.pose.position.y
        yaw_to_cart = math.atan2(cart_y - rear_y, cart_x - rear_x)
        goal = self.create_pose_stamped(rear_x, rear_y, yaw_to_cart)

        self.get_logger().info("sending rear heading alignment goal.")
        if not self.send_rear_goal(goal, "REAR_ALIGN", self.current_behavior_tree()):
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
                self.pending_precise_pose_msg = msg
                self.pending_precise_pose_type = "PoseStamped"
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
                self.pending_precise_pose_msg = msg
                self.pending_precise_pose_type = "Pose2D"
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

    def handle_precise_cart_pose(self, cart_x, cart_y, cart_yaw, source):
        if self.state != State.WAIT_PRECISE_POSE:
            return
        if not all(math.isfinite(value) for value in (cart_x, cart_y, cart_yaw)):
            self.get_logger().warn("precise cart pose contains invalid values.")
            return

        self.cancel_timer("precise_pose_timeout")
        cart_yaw = self.normalize_angle(cart_yaw)
        offset = self.dock_goal_offset

        rear_goal = self.create_pose_stamped(
            cart_x - offset * math.cos(cart_yaw),
            cart_y - offset * math.sin(cart_yaw),
            cart_yaw,
        )
        front_goal = self.create_pose_stamped(
            cart_x + offset * math.cos(cart_yaw),
            cart_y + offset * math.sin(cart_yaw),
            self.normalize_angle(cart_yaw + math.pi),
        )

        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.publish_dock_prep_done(False)
        self.set_state(State.DOCK_PREP, f"precise_pose:{source}")
        self.get_logger().info(
            "dock prep goals from cart=(%.2f, %.2f, %.2f)"
            % (cart_x, cart_y, cart_yaw)
        )

        rear_sent = self.send_rear_goal(rear_goal, "DOCK_PREP_REAR", self.diff_bt_path)
        front_sent = self.send_front_goal(front_goal, "DOCK_PREP_FRONT")
        if not rear_sent or not front_sent:
            self.abort_scenario("failed to send dock prep goals")

    def enter_wait_attach(self):
        self.clear_precise_pose()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.set_state(State.WAIT_ATTACH, "dock_prep_done")

        if self.release_to_joystick_on_wait_attach:
            self.enable_joystick_control()

        if self.pause_rear_nav2_on_dock_prep_done:
            self.pause_rear_nav2("dock_prep_done")

        if self.auto_attach_after_dock_prep:
            self.publish_bool_burst(
                "front_robot_docking",
                self.front_robot_docking_pub,
                True,
            )
            self.publish_bool_burst("gripper_grip", self.gripper_toggle_pub, True)

        if self.is_attached:
            self.complete_scenario("already attached")
            return

        if self.attach_timeout_sec > 0.0:
            self.schedule_once(
                "attach_timeout",
                self.attach_timeout_sec,
                self.handle_attach_timeout,
            )

        self.get_logger().info(
            "dock prep complete. Waiting for joystick/controller to publish "
            "/docking_state=true."
        )

    # ------------------------------------------------------------------
    # Route and action callbacks
    # ------------------------------------------------------------------
    def start_route(self, points, route_type, final_yaw=None):
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
        self.route_goal_retry_count = 0
        return self.send_current_route_goal()

    def send_current_route_goal(self):
        if self.active_route_index >= len(self.active_route_poses):
            self.finish_route()
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
        return True

    def finish_route(self):
        route_type = self.active_route_type
        self.clear_route()

        if route_type == "PATROL":
            self.complete_scenario("patrol complete")
        elif route_type == "EXIT":
            self.start_detach_sequence()

    def send_rear_goal(self, pose: PoseStamped, label: str, behavior_tree: str):
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

        if label in ("ROUTE_PATROL", "ROUTE_EXIT"):
            if not self.route_goal_reached_by_tf():
                self.handle_route_failure("route waypoint TF verification failed")
                return
            if self.active_route_type == "PATROL":
                self.current_patrol_waypoint_index = max(
                    self.current_patrol_waypoint_index,
                    self.active_route_index + 1,
                )
            self.route_goal_retry_count = 0
            self.active_route_index += 1
            self.send_current_route_goal()
        elif label == "REAR_ALIGN":
            self.wait_for_precise_pose()
        elif label == "DOCK_PREP_REAR":
            self.rear_dock_goal_done = True
            self.check_dock_prep_done()
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
            self.start_rear_heading_alignment()
        elif label == "DOCK_PREP_FRONT":
            self.front_dock_goal_done = True
            self.check_dock_prep_done()

    def check_dock_prep_done(self):
        if (
            self.state == State.DOCK_PREP
            and self.rear_dock_goal_done
            and self.front_dock_goal_done
        ):
            self.publish_dock_prep_done(True)
            self.enter_wait_attach()

    def handle_rear_goal_failure(self, reason):
        label = self.active_rear_goal_label
        self.active_rear_goal_handle = None
        self.active_rear_goal_pending = False

        if label in ("ROUTE_PATROL", "ROUTE_EXIT"):
            self.handle_route_failure(reason)
        elif label == "REAR_ALIGN":
            self.retry_or_abort("rear_align_goal_retry", self.start_rear_heading_alignment)
        elif label in ("DOCK_PREP_REAR", "MANUAL"):
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
            self.retry_or_abort("front_clear_goal_retry", self.start_front_clear_move)
        elif label == "DOCK_PREP_FRONT":
            self.abort_scenario(reason)
        else:
            self.get_logger().warn(reason)

    def schedule_front_goal_timeout(self):
        if self.front_goal_timeout_sec <= 0.0:
            return
        self.schedule_once(
            "front_goal_timeout",
            self.front_goal_timeout_sec,
            self.handle_front_goal_timeout,
        )

    def handle_front_goal_timeout(self):
        if self.active_front_goal_label is None:
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
        if self.route_goal_retry_count > self.route_goal_max_retries:
            self.abort_scenario(reason)
            return

        self.get_logger().warn(
            "%s. retry route goal %d/%d"
            % (reason, self.route_goal_retry_count, self.route_goal_max_retries)
        )
        self.schedule_once(
            "route_goal_retry",
            self.route_goal_retry_delay_sec,
            self.send_current_route_goal,
        )

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
            self.abort_scenario("detach timeout waiting for docking_state=false")

    def handle_precise_pose_timeout(self):
        if self.state == State.WAIT_PRECISE_POSE:
            self.abort_scenario("precise pose timeout")

    def handle_attach_timeout(self):
        if self.state == State.WAIT_ATTACH:
            self.abort_scenario("attach timeout waiting for docking_state=true")

    def complete_scenario(self, reason):
        self.get_logger().info(f"scenario complete: {reason}")
        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.publish_dock_prep_done(False)
        self.update_dynamic_state("scenario_complete")
        self.enable_joystick_control()
        self.set_state(State.IDLE, reason)

    def abort_scenario(self, reason, return_joystick=True):
        self.get_logger().error(f"scenario aborted: {reason}")
        self.cancel_all_timers()
        self.cancel_rear_goal()
        self.cancel_front_goal()
        self.clear_route()
        self.clear_precise_pose()
        self.publish_dock_prep_done(False)
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

    def enable_navigation_control(self):
        self.publish_bool_burst("rear_nav_control", self.rear_joy_sig_pub, False)
        self.publish_bool_burst("front_nav_control", self.front_joy_sig_pub, False)

    def enable_joystick_control(self):
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
        msg = Bool()
        msg.data = bool(done)
        self.dock_prep_done_pub.publish(msg)

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
        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0
        self.route_goal_retry_count = 0

    def clean_points(self, points):
        clean = []
        for x, y in points:
            if math.isfinite(x) and math.isfinite(y):
                clean.append((float(x), float(y)))
        return clean

    def create_path_poses(self, points, final_yaw=None):
        poses = []
        for index, (x, y) in enumerate(points):
            if index < len(points) - 1:
                next_x, next_y = points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif final_yaw is not None:
                yaw = final_yaw
            elif index > 0:
                prev_x, prev_y = points[index - 1]
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

    def pose_to_map_xy(self, pose, source_frame, label):
        if not (math.isfinite(pose.position.x) and math.isfinite(pose.position.y)):
            self.get_logger().warn(f"{label} has invalid x/y.")
            return None

        source_frame = self.normalize_frame_id(source_frame) or "map"
        if source_frame == "map":
            return (float(pose.position.x), float(pose.position.y))

        try:
            transform = self.tf_buffer.lookup_transform("map", source_frame, Time())
            map_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"{label} TF failed: {ex}")
            return None

        return (float(map_pose.position.x), float(map_pose.position.y))

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
            ax, ay = self.patrol_path[index]
            bx, by = self.patrol_path[index + 1]
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

        last_x, last_y = self.patrol_path[-1]
        prev_x, prev_y = self.patrol_path[-2]
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
        cumulative = self.path_cumulative_lengths()
        route = []

        for index, waypoint in enumerate(self.patrol_path):
            waypoint_progress = cumulative[index]
            if current_progress + 0.05 < waypoint_progress < exit_progress - 0.05:
                route.append(waypoint)

        if not route or math.hypot(route[-1][0] - exit_point[0], route[-1][1] - exit_point[1]) > 0.05:
            route.append(exit_point)

        return route

    def path_cumulative_lengths(self):
        cumulative = [0.0]
        for index in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[index]
            bx, by = self.patrol_path[index + 1]
            cumulative.append(cumulative[-1] + math.hypot(bx - ax, by - ay))
        return cumulative

    def waypoint_progress(self, waypoint_index):
        if waypoint_index < 0:
            return 0.0
        cumulative = self.path_cumulative_lengths()
        return cumulative[min(waypoint_index, len(cumulative) - 1)]

    def pose2d_to_map_pose(self, msg: Pose2D):
        frame_id = self.precise_pose2d_frame
        pose_stamped = self.create_pose_stamped(msg.x, msg.y, msg.theta, frame_id)
        if not frame_id or frame_id == "map":
            return pose_stamped.pose

        transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
        return tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)

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

    def clear_precise_pose(self):
        self.pending_precise_pose_msg = None
        self.pending_precise_pose_type = None
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
