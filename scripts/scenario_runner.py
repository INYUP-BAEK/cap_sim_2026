#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PointStamped, Polygon, Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Empty, UInt16

from ament_index_python.packages import get_package_share_directory
import tf2_geometry_msgs
import tf2_ros


class AutoNavCommander(Node):
    def __init__(self):
        super().__init__("auto_nav_commander")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.front_nav_client = ActionClient(
            self,
            NavigateToPose,
            "/front/navigate_to_pose",
        )

        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_nav_tree.xml")
        self.ackermann_cart2_bt_path = os.path.join(
            pkg_dir,
            "bt_xml",
            "ackermann_cart2_nav_tree.xml",
        )

        self.is_attached = True
        self.cart_count = 0
        self.robot_state = "IDLE"

        self.active_nav_goal_handle = None
        self.active_nav_goal_seq = 0
        self.active_nav_goal_pending = False

        self.active_front_goal_handle = None
        self.active_front_goal_seq = 0
        self.active_front_goal_pending = False

        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0
        self.current_patrol_waypoint_index = 0

        self.cart_final_goal_pose = None
        self.detected_cart_pose = None
        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False

        self.patrol_path = [
            (7.6, -16.34),
            (3.02, -0.34),
            (-8.76, -4.17),
            (-4.7, -20.33),
            (7.66, -16.14),
        ]

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
        self.rear_joy_sig_pub = self.create_publisher(Bool, "/joy_control_sig", 10)
        self.front_joy_sig_pub = self.create_publisher(Bool, "/front/joy_control_sig", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.docking_sub = self.create_subscription(
            Bool,
            "/docking_state",
            self.docking_callback,
            10,
        )
        self.cart_count_sub = self.create_subscription(
            UInt16,
            "/cart_count",
            self.cart_count_callback,
            10,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/mission_goal",
            self.mission_goal_callback,
            10,
        )
        self.cart_target_sub = self.create_subscription(
            PointStamped,
            "/vision/cart_target_ground",
            self.cart_target_callback,
            10,
        )
        self.precise_cart_pose_sub = self.create_subscription(
            PoseStamped,
            "/vision/cart_precise_pose",
            self.precise_cart_pose_callback,
            10,
        )
        self.precise_cart_pose2d_sub = self.create_subscription(
            Pose2D,
            "/vision/cart_precise_pose_2d",
            self.precise_cart_pose2d_callback,
            10,
        )
        self.mission_start_sub = self.create_subscription(
            Empty,
            "/start_patrol_mission",
            self.start_mission_callback,
            10,
        )

        self.get_logger().info("🤖 [Scenario Runner] 실행 완료! 임무 대기 중입니다.")

    def cart_count_callback(self, msg: UInt16):
        if self.cart_count == msg.data:
            return

        self.cart_count = msg.data
        self.get_logger().info(f"📊 카트 수 업데이트: {self.cart_count}대")
        self.update_dynamic_state()

    def docking_callback(self, msg: Bool):
        if self.is_attached == msg.data:
            return

        self.is_attached = msg.data
        self.get_logger().info(f"🔗 결합 상태 업데이트: {self.is_attached}")
        self.update_dynamic_state()

        if self.robot_state == "DETACHING" and not self.is_attached:
            self.start_front_clear_move()

    def update_dynamic_state(self):
        self.update_nav2_footprint()

        if self.is_attached:
            if self.cart_count >= 1:
                smoother_params = {
                    "max_velocity": [0.18, 0.0, 0.55],
                    "min_velocity": [-0.1, 0.0, -0.55],
                    "max_accel": [0.12, 0.0, 0.25],
                    "max_decel": [-0.15, 0.0, -0.3],
                }
                mode_str = f"아커만(카트 {self.cart_count}대) - 프론트 카메라 활성화"
            else:
                smoother_params = {
                    "max_velocity": [0.25, 0.0, 1.2],
                    "min_velocity": [-0.15, 0.0, -1.2],
                    "max_accel": [0.3, 0.0, 1.2],
                    "max_decel": [-0.5, 0.0, -1.2],
                }
                mode_str = "아커만(직결) - 프론트 카메라 활성화"
        else:
            smoother_params = {
                "max_velocity": [0.35, 0.0, 1.0],
                "min_velocity": [-0.35, 0.0, -1.0],
                "max_accel": [0.5, 0.0, 1.5],
                "max_decel": [-0.5, 0.0, -1.5],
            }
            mode_str = "디퍼런셜(분리) - 리어 카메라 활성화"

        self._send_parameters_to_node("/velocity_smoother", smoother_params)
        self.get_logger().info(f"🔄 [다이내믹 업데이트 완료] {mode_str}")

    def update_nav2_footprint(self):
        rear_width = 0.25
        rear_bumper_x = -0.3

        if self.is_attached:
            current_wheelbase = (
                0.48
                if self.cart_count == 0
                else 1.30 + (self.cart_count - 1) * 0.15
            )
            rear_msg = self._create_polygon(current_wheelbase + 0.3, rear_bumper_x, rear_width)
            front_msg = self._create_polygon(0.01, -0.01, 0.01)
        else:
            rear_msg = self._create_polygon(0.3, rear_bumper_x, rear_width)
            front_msg = self._create_polygon(0.3, -0.3, 0.25)

        self.global_footprint_pub.publish(rear_msg)
        self.local_footprint_pub.publish(rear_msg)
        self.front_global_footprint_pub.publish(front_msg)
        self.front_local_footprint_pub.publish(front_msg)

    def start_mission_callback(self, msg: Empty):
        if self.robot_state != "IDLE":
            self.get_logger().warn("⚠️ 현재 다른 주행 임무를 수행 중입니다. 순찰 명령 무시.")
            return

        self.get_logger().info("🚀 [미션 가동] 순차 waypoint 순찰 주행을 시작합니다!")
        self.enable_navigation_control()
        self.current_patrol_waypoint_index = 0
        self.robot_state = "PATROL"
        self._start_route(self.patrol_path, "PATROL")

    def mission_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"📥 수동 목적지 수신: X={msg.pose.position.x:.2f}, "
            f"Y={msg.pose.position.y:.2f}"
        )
        self._clear_route()
        self._cancel_active_nav_goal()
        self._cancel_active_front_goal()
        self.robot_state = "IDLE"
        self._send_nav_goal(msg)

    def cart_target_callback(self, msg: PointStamped):
        if self.robot_state != "PATROL":
            return

        if not msg.header.frame_id:
            self.get_logger().warn("카트 좌표 frame_id가 비어 있어 무시합니다.")
            return

        if self.active_nav_goal_handle is None and self.active_nav_goal_pending:
            self.get_logger().warn("순찰 goal이 아직 accept되지 않아 카트 목표 처리를 잠시 보류합니다.")
            return

        current_time = self.get_clock().now()
        cart_x, cart_y = msg.point.x, msg.point.y
        if not (math.isfinite(cart_x) and math.isfinite(cart_y)):
            self.get_logger().warn("카트 좌표가 유효하지 않아 무시합니다.")
            return

        distance_to_cart = math.hypot(cart_x, cart_y)
        stop_distance = 1.5 if self.is_attached else 1.0
        if distance_to_cart <= stop_distance:
            return

        yaw_to_cart = math.atan2(cart_y, cart_x)
        ratio = (distance_to_cart - stop_distance) / distance_to_cart

        local_goal_pose = PoseStamped()
        local_goal_pose.header.frame_id = msg.header.frame_id
        local_goal_pose.header.stamp = current_time.to_msg()
        local_goal_pose.pose.position.x = cart_x * ratio
        local_goal_pose.pose.position.y = cart_y * ratio
        local_goal_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_goal_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        local_cart_pose = PoseStamped()
        local_cart_pose.header.frame_id = msg.header.frame_id
        local_cart_pose.header.stamp = current_time.to_msg()
        local_cart_pose.pose.position.x = cart_x
        local_cart_pose.pose.position.y = cart_y
        local_cart_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_cart_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        try:
            transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, Time())
            cart_goal_pose = tf2_geometry_msgs.do_transform_pose(local_goal_pose.pose, transform)
            cart_map_pose = tf2_geometry_msgs.do_transform_pose(local_cart_pose.pose, transform)

            cart_global_x = cart_map_pose.position.x
            cart_global_y = cart_map_pose.position.y
            self.get_logger().info(
                f"👀 카트 발견! (글로벌 좌표 X={cart_global_x:.2f}, "
                f"Y={cart_global_y:.2f})"
            )

            final_pose = PoseStamped()
            final_pose.header.frame_id = "map"
            final_pose.header.stamp = current_time.to_msg()
            final_pose.pose = cart_goal_pose
            self.cart_final_goal_pose = final_pose

            detected_pose = PoseStamped()
            detected_pose.header.frame_id = "map"
            detected_pose.header.stamp = current_time.to_msg()
            detected_pose.pose = cart_map_pose
            self.detected_cart_pose = detected_pose

            current_progress = self.get_current_progress_on_path()
            exit_projection = self.get_closest_point_on_path(
                cart_global_x,
                cart_global_y,
                min_progress=current_progress + 0.05,
            )
            exit_x, exit_y = exit_projection["point"]
            self.get_logger().info(
                f"📍 경로상 최근접 이탈점(X={exit_x:.2f}, Y={exit_y:.2f})까지 waypoint 경로를 유지합니다."
            )

            self._clear_route()
            self._cancel_active_nav_goal()
            exit_route_points = self.build_waypoint_route_to_exit(
                exit_projection,
                current_progress,
            )
            self.robot_state = "APPROACH_EXIT"
            self._start_route(
                exit_route_points,
                "EXIT",
                final_yaw=exit_projection["path_yaw"],
            )

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"TF 변환 실패(로컬라이제이션 대기 중): {ex}")
        except Exception as ex:
            self.get_logger().error(f"카트 목표 처리 중 예외 발생: {ex}")
            self.robot_state = "PATROL"

    def _start_route(self, points, route_type, final_yaw=None):
        poses = self._create_path_poses(points, final_yaw=final_yaw)
        if not poses:
            self.get_logger().error("Route goal이 비어 있어 주행을 시작할 수 없습니다.")
            self.robot_state = "IDLE"
            return False

        self.active_route_type = route_type
        self.active_route_poses = poses
        self.active_route_index = 0
        return self._send_current_route_pose()

    def _send_current_route_pose(self):
        if self.active_route_index >= len(self.active_route_poses):
            self._finish_active_route()
            return True

        pose = self.active_route_poses[self.active_route_index]
        pose.header.stamp = self.get_clock().now().to_msg()

        if self.active_route_type == "PATROL":
            self.get_logger().info(
                f"📍 순찰 포인트 {self.active_route_index + 1}/{len(self.active_route_poses)} 로 이동합니다."
            )
        elif self.active_route_type == "EXIT":
            self.get_logger().info(
                f"📍 이탈점 경로 {self.active_route_index + 1}/{len(self.active_route_poses)} 로 이동합니다."
            )

        if not self._send_nav_goal(pose):
            self._clear_route()
            self.robot_state = "IDLE"
            return False

        return True

    def _finish_active_route(self):
        route_type = self.active_route_type
        self._clear_route()

        if route_type == "PATROL":
            self.get_logger().info("🏁 모든 순찰 경로를 탐색 완료했습니다.")
            self.robot_state = "IDLE"
        elif route_type == "EXIT":
            self.get_logger().info("✅ 경로상 이탈점 도착 완료! 분리 및 결합 준비 시퀀스를 시작합니다.")
            self.start_detach_sequence()

    def _clear_route(self):
        self.active_route_type = None
        self.active_route_poses = []
        self.active_route_index = 0

    def start_detach_sequence(self):
        self.robot_state = "DETACHING"
        self.enable_navigation_control()

        self.get_logger().info("🔓 이탈점 도착: 그리퍼 해제 및 프론트봇 home 동작으로 분리를 시작합니다.")
        self._publish_bool(self.gripper_toggle_pub, False)
        self._publish_bool(self.front_home_pub, True)

        if not self.is_attached:
            self.start_front_clear_move()

    def start_front_clear_move(self):
        if self.robot_state not in ("DETACHING", "FRONT_CLEARING"):
            return

        front_pose = self.get_robot_pose_in_map(("front/base_footprint", "front/base_link"))
        if front_pose is None:
            self.get_logger().error("프론트봇 TF를 찾지 못해 50cm 전진 목표를 만들 수 없습니다.")
            self.robot_state = "IDLE"
            return

        x, y, yaw = front_pose
        clear_distance = 0.5
        goal = self._create_pose_stamped(
            x + clear_distance * math.cos(yaw),
            y + clear_distance * math.sin(yaw),
            yaw,
        )

        self.robot_state = "FRONT_CLEARING"
        self.get_logger().info(
            f"↗️ 프론트봇 분리 여유 확보: 현재 heading 기준 "
            f"{clear_distance:.2f}m 전진 목표를 보냅니다."
        )
        if not self._send_front_nav_goal(goal):
            self.robot_state = "IDLE"

    def start_rear_heading_alignment(self):
        if self.detected_cart_pose is None:
            self.get_logger().error("카트 추정 좌표가 없어 리어봇 heading 정렬을 시작할 수 없습니다.")
            self.robot_state = "IDLE"
            return

        rear_pose = self.get_robot_pose_in_map(("base_footprint", "base_link", "rear_base_link"))
        if rear_pose is None:
            self.get_logger().error("리어봇 TF를 찾지 못해 heading 정렬 목표를 만들 수 없습니다.")
            self.robot_state = "IDLE"
            return

        rear_x, rear_y, _ = rear_pose
        cart_x = self.detected_cart_pose.pose.position.x
        cart_y = self.detected_cart_pose.pose.position.y
        yaw_to_cart = math.atan2(cart_y - rear_y, cart_x - rear_x)
        goal = self._create_pose_stamped(rear_x, rear_y, yaw_to_cart)

        self.robot_state = "REAR_ALIGNING"
        self.get_logger().info("🎯 리어봇을 카트 방향으로 회전시켜 정밀 자세 추정을 준비합니다.")
        if not self._send_nav_goal(goal):
            self.robot_state = "IDLE"

    def precise_cart_pose_callback(self, msg: PoseStamped):
        if self.robot_state != "WAIT_PRECISE_CART_POSE":
            return

        try:
            if msg.header.frame_id and msg.header.frame_id != "map":
                transform = self.tf_buffer.lookup_transform("map", msg.header.frame_id, Time())
                cart_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            else:
                cart_pose = msg.pose
        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"정밀 카트 pose TF 변환 실패: {ex}")
            return
        except Exception as ex:
            self.get_logger().error(f"정밀 카트 pose 처리 중 예외 발생: {ex}")
            return

        yaw = self.yaw_from_quaternion(cart_pose.orientation)
        self.handle_precise_cart_pose(
            cart_pose.position.x,
            cart_pose.position.y,
            yaw,
            "PoseStamped",
        )

    def precise_cart_pose2d_callback(self, msg: Pose2D):
        if self.robot_state != "WAIT_PRECISE_CART_POSE":
            return

        self.handle_precise_cart_pose(msg.x, msg.y, msg.theta, "Pose2D")

    def handle_precise_cart_pose(self, cart_x, cart_y, cart_yaw, source):
        if not all(math.isfinite(v) for v in (cart_x, cart_y, cart_yaw)):
            self.get_logger().warn("정밀 카트 pose에 유효하지 않은 값이 있어 무시합니다.")
            return

        cart_yaw = self.normalize_angle(cart_yaw)
        offset = 1.0

        rear_goal_x = cart_x - offset * math.cos(cart_yaw)
        rear_goal_y = cart_y - offset * math.sin(cart_yaw)
        rear_goal_yaw = cart_yaw

        front_goal_x = cart_x + offset * math.cos(cart_yaw)
        front_goal_y = cart_y + offset * math.sin(cart_yaw)
        front_goal_yaw = self.normalize_angle(cart_yaw + math.pi)

        rear_goal = self._create_pose_stamped(rear_goal_x, rear_goal_y, rear_goal_yaw)
        front_goal = self._create_pose_stamped(front_goal_x, front_goal_y, front_goal_yaw)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("리어 Nav2 서버가 응답하지 않아 도킹 준비 goal을 보내지 않습니다.")
            return

        if not self.front_nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "프론트 Nav2 서버(/front/navigate_to_pose)가 응답하지 않아 "
                "도킹 준비 goal을 보내지 않습니다."
            )
            return

        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.robot_state = "DOCK_GOALS_ACTIVE"

        self.get_logger().info(
            f"📐 정밀 카트 pose 수신({source}): "
            f"cart=({cart_x:.2f}, {cart_y:.2f}, yaw={cart_yaw:.2f})"
        )
        self.get_logger().info(
            f"🚚 리어 goal=({rear_goal_x:.2f}, {rear_goal_y:.2f}), "
            f"프론트 goal=({front_goal_x:.2f}, {front_goal_y:.2f})"
        )

        rear_sent = self._send_nav_goal(rear_goal)
        front_sent = self._send_front_nav_goal(front_goal)

        if not rear_sent or not front_sent:
            self.get_logger().error("프론트/리어 도킹 준비 goal 전송 실패. 시퀀스를 중단합니다.")
            self._cancel_active_nav_goal()
            self._cancel_active_front_goal()
            self.robot_state = "IDLE"

    def check_dock_goal_completion(self):
        if (
            self.robot_state == "DOCK_GOALS_ACTIVE"
            and self.rear_dock_goal_done
            and self.front_dock_goal_done
        ):
            self.get_logger().info("✅ 프론트/리어봇이 카트 결합 준비 위치에 모두 도착했습니다.")
            self.robot_state = "IDLE"

    def get_current_progress_on_path(self):
        fallback_progress = self.get_waypoint_progress(self.current_patrol_waypoint_index - 1)
        robot_xy = self.get_robot_xy_in_map()
        if robot_xy is None:
            return fallback_progress

        projection = self.get_closest_point_on_path(
            robot_xy[0],
            robot_xy[1],
            min_progress=fallback_progress,
        )
        return projection["progress"]

    def get_robot_xy_in_map(self):
        robot_pose = self.get_robot_pose_in_map(("base_link", "base_footprint", "rear_base_link"))
        if robot_pose is not None:
            return (robot_pose[0], robot_pose[1])

        self.get_logger().warn("로봇 base TF를 찾지 못해 waypoint index 기준으로 진행도를 추정합니다.")
        return None

    def get_robot_pose_in_map(self, frame_candidates):
        for frame_id in frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform("map", frame_id, Time())
                translation = transform.transform.translation
                yaw = self.yaw_from_quaternion(transform.transform.rotation)
                return (translation.x, translation.y, yaw)
            except tf2_ros.TransformException:
                continue
        return None

    def get_closest_point_on_path(self, target_x, target_y, min_progress=0.0):
        min_dist = float("inf")
        best_projection = None
        cumulative = self.get_path_cumulative_lengths()

        for i in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[i]
            bx, by = self.patrol_path[i + 1]
            ab_dx, ab_dy = bx - ax, by - ay
            ab_len_sq = ab_dx**2 + ab_dy**2
            if ab_len_sq == 0.0:
                continue

            ab_len = math.sqrt(ab_len_sq)
            t = max(
                0.0,
                min(
                    1.0,
                    ((target_x - ax) * ab_dx + (target_y - ay) * ab_dy)
                    / ab_len_sq,
                ),
            )
            progress = cumulative[i] + t * ab_len
            if progress + 1e-6 < min_progress:
                continue

            closest_x = ax + t * ab_dx
            closest_y = ay + t * ab_dy
            dist = math.hypot(target_x - closest_x, target_y - closest_y)
            if dist < min_dist:
                min_dist = dist
                best_projection = {
                    "point": (closest_x, closest_y),
                    "segment_index": i,
                    "t": t,
                    "progress": progress,
                    "distance": dist,
                    "path_yaw": math.atan2(ab_dy, ab_dx),
                }

        if best_projection is not None:
            return best_projection

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

    def build_waypoint_route_to_exit(self, exit_projection, current_progress):
        exit_progress = exit_projection["progress"]
        exit_point = exit_projection["point"]
        cumulative = self.get_path_cumulative_lengths()
        route_points = []

        for i, waypoint in enumerate(self.patrol_path):
            waypoint_progress = cumulative[i]
            if current_progress + 0.05 < waypoint_progress < exit_progress - 0.05:
                route_points.append(waypoint)

        if (
            not route_points
            or math.hypot(
                route_points[-1][0] - exit_point[0],
                route_points[-1][1] - exit_point[1],
            )
            > 0.05
        ):
            route_points.append(exit_point)

        return route_points

    def get_path_cumulative_lengths(self):
        cumulative = [0.0]
        for i in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[i]
            bx, by = self.patrol_path[i + 1]
            cumulative.append(cumulative[-1] + math.hypot(bx - ax, by - ay))
        return cumulative

    def get_waypoint_progress(self, waypoint_index):
        if waypoint_index < 0:
            return 0.0

        cumulative = self.get_path_cumulative_lengths()
        return cumulative[min(waypoint_index, len(cumulative) - 1)]

    def _create_path_poses(self, points, final_yaw=None):
        now_msg = self.get_clock().now().to_msg()
        poses = []

        for i, (x, y) in enumerate(points):
            if i < len(points) - 1:
                next_x, next_y = points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif final_yaw is not None:
                yaw = final_yaw
            elif i > 0:
                prev_x, prev_y = points[i - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0

            pose = self._create_pose_stamped(x, y, yaw)
            pose.header.stamp = now_msg
            poses.append(pose)

        return poses

    def _create_pose_stamped(self, x, y, yaw, frame_id="map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_nav_goal(self, msg: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        if not self.is_attached:
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info("🌳 [자동 선택] 디퍼런셜 트리를 사용합니다.")
        elif self.cart_count >= 1:
            goal_msg.behavior_tree = self.ackermann_cart2_bt_path
            self.get_logger().info("🌳 [자동 선택] 아커만(카트) 트리를 사용합니다.")
        else:
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info("🌳 [자동 선택] 아커만(직결) 트리를 사용합니다.")

        self.active_nav_goal_seq += 1
        goal_seq = self.active_nav_goal_seq
        self.active_nav_goal_pending = True

        try:
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda fut, seq=goal_seq: self.nav_goal_response_callback(fut, seq)
            )
        except Exception as ex:
            self.active_nav_goal_pending = False
            self.get_logger().error(f"Nav2 목표 전송 중 예외 발생: {ex}")
            return False

        return True

    def _send_front_nav_goal(self, msg: PoseStamped):
        if not self.front_nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("프론트 Nav2 서버(/front/navigate_to_pose)가 응답하지 않습니다!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.active_front_goal_seq += 1
        goal_seq = self.active_front_goal_seq
        self.active_front_goal_pending = True

        try:
            future = self.front_nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda fut, seq=goal_seq: self.front_nav_goal_response_callback(
                    fut,
                    seq,
                )
            )
        except Exception as ex:
            self.active_front_goal_pending = False
            self.get_logger().error(f"프론트 Nav2 목표 전송 중 예외 발생: {ex}")
            return False

        return True

    def nav_goal_response_callback(self, future, goal_seq):
        if goal_seq != self.active_nav_goal_seq:
            return

        self.active_nav_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"Nav2 목표 응답 처리 중 예외 발생: {ex}")
            self.robot_state = "IDLE"
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2가 주행을 거부했거나 응답이 비어 있습니다.")
            self.robot_state = "IDLE"
            return

        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=goal_seq: self.nav_result_callback(fut, seq)
        )

    def front_nav_goal_response_callback(self, future, goal_seq):
        if goal_seq != self.active_front_goal_seq:
            return

        self.active_front_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"프론트 Nav2 목표 응답 처리 중 예외 발생: {ex}")
            self.robot_state = "IDLE"
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("프론트 Nav2가 주행을 거부했거나 응답이 비어 있습니다.")
            self.robot_state = "IDLE"
            return

        self.active_front_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=goal_seq: self.front_nav_result_callback(fut, seq)
        )

    def nav_result_callback(self, future, goal_seq):
        if goal_seq != self.active_nav_goal_seq:
            return

        self.active_nav_goal_handle = None
        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f"Nav2 결과 처리 중 예외 발생: {ex}")
            self.robot_state = "IDLE"
            return

        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = "None" if result is None else result.status
            self.get_logger().warn(f"Nav2 주행이 성공하지 못했습니다. status={status}")
            if self.robot_state == "DOCK_GOALS_ACTIVE":
                self._cancel_active_front_goal()
            self.robot_state = "IDLE"
            return

        if self.robot_state in ("PATROL", "APPROACH_EXIT"):
            if self.robot_state == "PATROL":
                self.current_patrol_waypoint_index = max(
                    self.current_patrol_waypoint_index,
                    self.active_route_index + 1,
                )

            self.active_route_index += 1
            self._send_current_route_pose()
        elif self.robot_state == "REAR_ALIGNING":
            self.get_logger().info("✅ 리어봇 heading 정렬 완료. 정밀 카트 pose 입력을 기다립니다.")
            self.robot_state = "WAIT_PRECISE_CART_POSE"
        elif self.robot_state == "DOCK_GOALS_ACTIVE":
            self.rear_dock_goal_done = True
            self.get_logger().info("✅ 리어봇이 카트 후방 결합 준비 위치에 도착했습니다.")
            self.check_dock_goal_completion()
        else:
            self.get_logger().info("✅ 일반 목적지에 무사히 도착했습니다!")
            self.robot_state = "IDLE"

    def front_nav_result_callback(self, future, goal_seq):
        if goal_seq != self.active_front_goal_seq:
            return

        self.active_front_goal_handle = None
        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f"프론트 Nav2 결과 처리 중 예외 발생: {ex}")
            self.robot_state = "IDLE"
            return

        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = "None" if result is None else result.status
            self.get_logger().warn(f"프론트 Nav2 주행이 성공하지 못했습니다. status={status}")
            if self.robot_state == "DOCK_GOALS_ACTIVE":
                self._cancel_active_nav_goal()
            self.robot_state = "IDLE"
            return

        if self.robot_state == "FRONT_CLEARING":
            self.get_logger().info("✅ 프론트봇 50cm 전진 완료.")
            self.start_rear_heading_alignment()
        elif self.robot_state == "DOCK_GOALS_ACTIVE":
            self.front_dock_goal_done = True
            self.get_logger().info("✅ 프론트봇이 카트 전방 결합 준비 위치에 도착했습니다.")
            self.check_dock_goal_completion()

    def _cancel_active_nav_goal(self):
        self.active_nav_goal_seq += 1
        self.active_nav_goal_pending = False
        if self.active_nav_goal_handle is not None:
            self.active_nav_goal_handle.cancel_goal_async()
            self.active_nav_goal_handle = None

    def _cancel_active_front_goal(self):
        self.active_front_goal_seq += 1
        self.active_front_goal_pending = False
        if self.active_front_goal_handle is not None:
            self.active_front_goal_handle.cancel_goal_async()
            self.active_front_goal_handle = None

    def _send_parameters_to_node(self, node_name, param_dict):
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"⚠️ {node_name} 서비스 연결 지연 중.")
            return

        request = SetParameters.Request()
        for name, value in param_dict.items():
            try:
                request.parameters.append(
                    Parameter(name, value=value).to_parameter_msg()
                )
            except Exception as ex:
                self.get_logger().error(f"❌ 파라미터 변환 실패 ({name}): {ex}")

        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, n=node_name: self._parameter_set_callback(fut, n)
        )

    def _parameter_set_callback(self, future: Future, node_name: str):
        try:
            response = future.result()
            failed = [res.reason for res in response.results if not res.successful]
            if failed:
                self.get_logger().warn(f"⚠️ {node_name} 파라미터 업데이트 거부됨: {failed}")
            else:
                self.get_logger().info(f"✅ {node_name} 파라미터 실시간 적용 완료!")
        except Exception as ex:
            self.get_logger().error(f"❌ {node_name} 통신 에러 발생: {ex}")

    def _create_polygon(self, front_x, rear_x, width):
        poly = Polygon()
        poly.points = [
            Point32(x=float(front_x), y=float(-width), z=0.0),
            Point32(x=float(front_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(-width), z=0.0),
        ]
        return poly

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_bool(self, publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def enable_navigation_control(self):
        self._publish_bool(self.rear_joy_sig_pub, False)
        self._publish_bool(self.front_joy_sig_pub, False)


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
