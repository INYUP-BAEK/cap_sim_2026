#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.task import Future
from rcl_interfaces.srv import SetParameters

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, Polygon, Point32, PointStamped, Pose2D
from std_msgs.msg import Bool, UInt16, Empty

from ament_index_python.packages import get_package_share_directory
import tf2_ros
import tf2_geometry_msgs


class AutoNavCommander(Node):
    def __init__(self):
        super().__init__("auto_nav_commander")

        # 1. Nav2 액션 클라이언트 및 BT XML 경로 설정 (3가지 트리 적용)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.front_nav_client = ActionClient(self, NavigateToPose, "/front/navigate_to_pose")
        self.wp_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        
        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_nav_tree.xml")
        self.ackermann_cart2_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_cart2_nav_tree.xml")

        # 2. 로봇 상태 및 주행 관리 변수 (State Machine)
        self.is_attached = True
        self.cart_count = 0
        
        self.robot_state = 'IDLE'
        self.active_wp_goal_handle = None   # waypoint 액션 취소용 핸들
        self.active_wp_goal_type = None     # PATROL 또는 EXIT
        self.active_wp_goal_seq = 0         # 취소된 waypoint 결과 콜백 무시용 토큰
        self.current_patrol_waypoint_index = 0
        self.cart_final_goal_pose = None    # 카트 최종 목적지 백업용
        self.detected_cart_pose = None      # waypoint 이탈점 계산에 사용한 카트 추정 pose
        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        
        # 🚨 [수정 필요] 로봇이 순찰할 커스텀 글로벌 경로 (X, Y) 리스트
        self.patrol_path = [
            (6.82, 2.5), (7.72, 2.45), (4.52, 1.95), (4.67, 1.1)]


        # 3. 풋프린트 퍼블리셔 설정
        self.global_footprint_pub = self.create_publisher(Polygon, "/global_costmap/footprint", 10)
        self.local_footprint_pub = self.create_publisher(Polygon, "/local_costmap/footprint", 10)
        self.front_global_footprint_pub = self.create_publisher(Polygon, '/front/global_costmap/footprint', 10)
        self.front_local_footprint_pub = self.create_publisher(Polygon, '/front/local_costmap/footprint', 10)
        self.gripper_toggle_pub = self.create_publisher(Bool, "/gripper_toggle", 10)
        self.front_home_pub = self.create_publisher(Bool, "/front/home", 10)
        self.rear_joy_sig_pub = self.create_publisher(Bool, "/joy_control_sig", 10)
        self.front_joy_sig_pub = self.create_publisher(Bool, "/front/joy_control_sig", 10)

        # 4. TF2 버퍼 및 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 구독(Subscriber) 설정
        self.docking_sub = self.create_subscription(Bool, "/docking_state", self.docking_callback, 10)
        self.cart_count_sub = self.create_subscription(UInt16, "/cart_count", self.cart_count_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/mission_goal", self.mission_goal_callback, 10)
        self.cart_target_sub = self.create_subscription(PointStamped, "/vision/cart_target_ground", self.cart_target_callback, 10)
        self.precise_cart_pose_sub = self.create_subscription(PoseStamped, "/vision/cart_precise_pose", self.precise_cart_pose_callback, 10)
        self.precise_cart_pose2d_sub = self.create_subscription(Pose2D, "/vision/cart_precise_pose_2d", self.precise_cart_pose2d_callback, 10)
        
        # [신규] 순찰 미션 시작 트리거
        self.mission_start_sub = self.create_subscription(Empty, "/start_patrol_mission", self.start_mission_callback, 10)

        self.get_logger().info("🤖 [Scenario Runner] 실행 완료! 임무 대기 중입니다.")

    # ----------------------------------------------------------
    # 🔄 상태 업데이트 콜백 함수
    # ----------------------------------------------------------
    def cart_count_callback(self, msg: UInt16):
        if self.cart_count != msg.data:
            self.cart_count = msg.data
            self.get_logger().info(f"📊 카트 수 업데이트: {self.cart_count}대")
            self.update_dynamic_state()

    def docking_callback(self, msg: Bool):
        if self.is_attached != msg.data:
            self.is_attached = msg.data
            self.get_logger().info(f"🔗 결합 상태 업데이트: {self.is_attached}")
            self.update_dynamic_state()

            if self.robot_state == 'DETACHING' and not self.is_attached:
                self.start_front_clear_move()

    def update_dynamic_state(self):
        """상태가 변할 때 풋프린트 토픽을 쏘고, 카메라 레이어 및 스무더 파라미터만 가볍게 변경합니다."""
        # 1. 동적 풋프린트 실시간 퍼블리시
        self.update_nav2_footprint()

        # 2. 필요한 파라미터만 딕셔너리로 전송
        if self.is_attached:
            if self.cart_count >= 1:
                # 🚛 [합체 - 카트 1개 이상] 무겁고 둔함
                smoother_params = {
                    'max_velocity': [0.18, 0.0, 0.55],
                    'min_velocity': [-0.1, 0.0, -0.55],
                    'max_accel': [0.12, 0.0, 0.25],
                    'max_decel': [-0.15, 0.0, -0.3]
                }
                mode_str = f"아커만(카트 {self.cart_count}대) - 프론트 카메라 활성화"
            else:
                # 🏎️ [합체 - 직결] 가볍고 빠름
                smoother_params = {
                    'max_velocity': [0.25, 0.0, 0.7],
                    'min_velocity': [-0.15, 0.0, -0.7],
                    'max_accel': [0.3, 0.0, 0.8],
                    'max_decel': [-0.5, 0.0, -1.0]
                }
                mode_str = "아커만(직결) - 프론트 카메라 활성화"
            
            camera_params = {
                'stvl_camera_rear_layer.enabled': False,
                'stvl_camera_front_layer.enabled': True
            }
        else:
            # ✂️ [분리 - 디퍼런셜]
            smoother_params = {
                'max_velocity': [0.35, 0.0, 1.0],
                'min_velocity': [-0.35, 0.0, -1.0],
                'max_accel': [0.5, 0.0, 1.5],
                'max_decel': [-0.5, 0.0, -1.5]
            }
            camera_params = {
                'stvl_camera_rear_layer.enabled': True,
                'stvl_camera_front_layer.enabled': False
            }
            mode_str = "디퍼런셜(분리) - 리어 카메라 활성화"

        # 코스트맵과 벨로시티 스무더에 파라미터 실시간 전송
        # self._send_parameters_to_node('/local_costmap/local_costmap', camera_params)
        # self._send_parameters_to_node('/global_costmap/global_costmap', camera_params)
        self._send_parameters_to_node('/velocity_smoother', smoother_params)

        self.get_logger().info(f"🔄 [다이내믹 업데이트 완료] {mode_str}")

    def update_nav2_footprint(self):
        """현재 상태에 맞춰 풋프린트 토픽을 발행합니다."""
        rear_width = 0.25
        rear_bumper_x = -0.3

        if self.is_attached:
            current_wheelbase = 0.48 if self.cart_count == 0 else 1.30 + (self.cart_count - 1) * 0.15
            rear_front_bumper_x = current_wheelbase + 0.3

            rear_msg = self._create_polygon(rear_front_bumper_x, rear_bumper_x, rear_width)
            tiny_size = 0.01
            front_msg = self._create_polygon(tiny_size, -tiny_size, tiny_size)
        else:
            rear_msg = self._create_polygon(0.3, rear_bumper_x, rear_width)
            front_msg = self._create_polygon(0.3, -0.3, 0.25)

        self.global_footprint_pub.publish(rear_msg)
        self.local_footprint_pub.publish(rear_msg)
        self.front_global_footprint_pub.publish(front_msg)
        self.front_local_footprint_pub.publish(front_msg)

    # ----------------------------------------------------------
    # 🏁 미션 제어 (순찰 시작 및 일반 목표 수신)
    # ----------------------------------------------------------
    def start_mission_callback(self, msg: Empty):
        """순찰 미션 시작 트리거"""
        if self.robot_state != 'IDLE':
            self.get_logger().warn("⚠️ 현재 다른 주행 임무를 수행 중입니다. 순찰 명령 무시.")
            return

        self.get_logger().info("🚀 [미션 가동] 글로벌 경로(Waypoint) 순찰 주행을 시작합니다!")
        self.enable_navigation_control()
        self.robot_state = 'PATROL'

        self.current_patrol_waypoint_index = 0
        waypoints = self._create_path_poses(self.patrol_path)
        if not self._send_waypoint_goal(waypoints, 'PATROL'):
            self.robot_state = 'IDLE'

    def mission_goal_callback(self, msg: PoseStamped):
        """단일 일반 목표 수신 시 (수동 제어 등)"""
        self.get_logger().info(f"📥 일반 목적지 수신: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        
        self._cancel_active_waypoint_goal()
            
        self.robot_state = 'IDLE' 
        self._send_nav_goal(msg)

    # ----------------------------------------------------------
    # 🛒 카트 타겟 수신 및 이탈점 계산 콜백
    # ----------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        """카트 발견 시 호출되어 이탈점 계산 후 궤도 수정"""
        if self.robot_state != 'PATROL':
            return

        if not msg.header.frame_id:
            self.get_logger().warn("카트 좌표 frame_id가 비어 있어 무시합니다.")
            return

        if self.active_wp_goal_handle is None:
            if self.active_wp_goal_type == 'PATROL':
                self.get_logger().warn("순찰 waypoint 목표가 아직 accept되지 않아 카트 목표 처리를 잠시 보류합니다.")
            else:
                self.get_logger().warn("PATROL 상태이지만 활성 waypoint goal이 없어 카트 목표를 무시합니다.")
            return

        current_time = self.get_clock().now()
        cart_x, cart_y = msg.point.x, msg.point.y
        if not (math.isfinite(cart_x) and math.isfinite(cart_y)):
            self.get_logger().warn("카트 좌표가 유효하지 않아 무시합니다.")
            return

        distance_to_cart = math.hypot(cart_x, cart_y)
        yaw_to_cart = math.atan2(cart_y, cart_x)

        stop_distance = 1.5 if self.is_attached else 1.0
        if distance_to_cart <= stop_distance:
            return

        ratio = (distance_to_cart - stop_distance) / distance_to_cart
        local_goal_x = cart_x * ratio
        local_goal_y = cart_y * ratio

        local_pose = PoseStamped()
        local_pose.header.frame_id = msg.header.frame_id
        local_pose.header.stamp = current_time.to_msg()
        local_pose.pose.position.x = local_goal_x
        local_pose.pose.position.y = local_goal_y
        local_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        cart_pose = PoseStamped()
        cart_pose.header.frame_id = msg.header.frame_id
        cart_pose.header.stamp = current_time.to_msg()
        cart_pose.pose.position.x = cart_x
        cart_pose.pose.position.y = cart_y
        cart_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        cart_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        cancelled_patrol = False

        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
            transformed_goal_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)
            transformed_cart_pose = tf2_geometry_msgs.do_transform_pose(cart_pose.pose, transform)
            cart_goal_pose = transformed_goal_pose.pose if hasattr(transformed_goal_pose, 'pose') else transformed_goal_pose
            cart_map_pose = transformed_cart_pose.pose if hasattr(transformed_cart_pose, 'pose') else transformed_cart_pose

            cart_global_x = cart_map_pose.position.x
            cart_global_y = cart_map_pose.position.y

            self.get_logger().info(f"👀 카트 발견! (글로벌 좌표 X={cart_global_x:.2f}, Y={cart_global_y:.2f})")

            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = current_time.to_msg()
            final_pose.pose = cart_goal_pose
            self.cart_final_goal_pose = final_pose

            detected_pose = PoseStamped()
            detected_pose.header.frame_id = 'map'
            detected_pose.header.stamp = current_time.to_msg()
            detected_pose.pose = cart_map_pose
            self.detected_cart_pose = detected_pose

            current_progress = self.get_current_progress_on_path()
            exit_projection = self.get_closest_point_on_path(
                cart_global_x,
                cart_global_y,
                min_progress=current_progress + 0.05
            )
            exit_x, exit_y = exit_projection['point']
            exit_yaw = math.atan2(cart_global_y - exit_y, cart_global_x - exit_x)

            exit_route_points = self.build_waypoint_route_to_exit(exit_projection, current_progress)
            exit_waypoints = self._create_path_poses(exit_route_points, final_yaw=exit_yaw)

            if self.active_wp_goal_handle is not None:
                self.get_logger().info("🛑 카트 접근 지점까지 남은 waypoint 경로를 재구성합니다.")
                self._cancel_active_waypoint_goal()
                cancelled_patrol = True

            self.robot_state = 'APPROACH_EXIT'
            self.get_logger().info(
                f"📍 경로상 최근접 이탈점(X={exit_x:.2f}, Y={exit_y:.2f})까지 waypoint를 따라 이동합니다."
            )
            if not self._send_waypoint_goal(exit_waypoints, 'EXIT'):
                self.robot_state = 'IDLE'

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"TF 변환 실패(로컬라이제이션 대기 중): {ex}")
        except Exception as ex:
            self.get_logger().error(f"카트 목표 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE' if cancelled_patrol else 'PATROL'

    def start_detach_sequence(self):
        """이탈점 도착 후 프론트/리어 분리 절차 시작."""
        self.robot_state = 'DETACHING'
        self.enable_navigation_control()

        self.get_logger().info("🔓 이탈점 도착: 그리퍼 해제 및 프론트봇 home 동작으로 분리를 시작합니다.")
        self._publish_bool(self.gripper_toggle_pub, False)
        self._publish_bool(self.front_home_pub, True)

        if not self.is_attached:
            self.start_front_clear_move()

    def start_front_clear_move(self):
        """분리 완료 후 프론트봇을 전방으로 50cm 이동."""
        if self.robot_state not in ('DETACHING', 'FRONT_CLEARING'):
            return

        front_pose = self.get_robot_pose_in_map(('front/base_footprint', 'front/base_link'))
        if front_pose is None:
            self.get_logger().error("프론트봇 TF를 찾지 못해 50cm 전진 목표를 만들 수 없습니다.")
            self.robot_state = 'IDLE'
            return

        x, y, yaw = front_pose
        clear_distance = 0.5
        goal = self._create_pose_stamped(
            x + clear_distance * math.cos(yaw),
            y + clear_distance * math.sin(yaw),
            yaw
        )

        self.robot_state = 'FRONT_CLEARING'
        self.get_logger().info(
            f"↗️ 프론트봇 분리 여유 확보: 현재 heading 기준 {clear_distance:.2f}m 전진 목표를 보냅니다."
        )

        if not self._send_front_nav_goal(goal):
            self.robot_state = 'IDLE'

    def start_rear_heading_alignment(self):
        """리어봇이 카트 방향을 바라보도록 현재 위치에서 heading만 정렬."""
        if self.detected_cart_pose is None:
            self.get_logger().error("카트 추정 좌표가 없어 리어봇 heading 정렬을 시작할 수 없습니다.")
            self.robot_state = 'IDLE'
            return

        rear_pose = self.get_robot_pose_in_map(('base_footprint', 'base_link', 'rear_base_link'))
        if rear_pose is None:
            self.get_logger().error("리어봇 TF를 찾지 못해 heading 정렬 목표를 만들 수 없습니다.")
            self.robot_state = 'IDLE'
            return

        rear_x, rear_y, _ = rear_pose
        cart_x = self.detected_cart_pose.pose.position.x
        cart_y = self.detected_cart_pose.pose.position.y
        yaw_to_cart = math.atan2(cart_y - rear_y, cart_x - rear_x)

        goal = self._create_pose_stamped(rear_x, rear_y, yaw_to_cart)

        self.robot_state = 'REAR_ALIGNING'
        self.get_logger().info("🎯 리어봇을 카트 방향으로 회전시켜 정밀 자세 추정을 준비합니다.")

        if not self._send_nav_goal(goal):
            self.robot_state = 'IDLE'

    def precise_cart_pose_callback(self, msg: PoseStamped):
        """정밀 카트 pose(PoseStamped) 수신: frame_id가 map이 아니면 map으로 변환."""
        if self.robot_state != 'WAIT_PRECISE_CART_POSE':
            return

        try:
            if msg.header.frame_id and msg.header.frame_id != 'map':
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
                cart_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            else:
                cart_pose = msg.pose
        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"정밀 카트 pose TF 변환 실패: {ex}")
            return
        except Exception as ex:
            self.get_logger().error(f"정밀 카트 pose 처리 중 예외 발생: {ex}")
            return

        x = cart_pose.position.x
        y = cart_pose.position.y
        yaw = self.yaw_from_quaternion(cart_pose.orientation)
        self.handle_precise_cart_pose(x, y, yaw, "PoseStamped")

    def precise_cart_pose2d_callback(self, msg: Pose2D):
        """정밀 카트 pose(Pose2D) 수신: map frame 기준으로 해석."""
        if self.robot_state != 'WAIT_PRECISE_CART_POSE':
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

        self.rear_dock_goal_done = False
        self.front_dock_goal_done = False
        self.robot_state = 'DOCK_GOALS_ACTIVE'

        self.get_logger().info(
            f"📐 정밀 카트 pose 수신({source}): cart=({cart_x:.2f}, {cart_y:.2f}, yaw={cart_yaw:.2f})"
        )
        self.get_logger().info(
            f"🚚 리어 goal=({rear_goal_x:.2f}, {rear_goal_y:.2f}), 프론트 goal=({front_goal_x:.2f}, {front_goal_y:.2f})"
        )

        rear_sent = self._send_nav_goal(rear_goal)
        front_sent = self._send_front_nav_goal(front_goal)

        if not rear_sent or not front_sent:
            self.get_logger().error("프론트/리어 도킹 준비 goal 전송 실패. 시퀀스를 중단합니다.")
            self.robot_state = 'IDLE'

    def check_dock_goal_completion(self):
        if self.robot_state == 'DOCK_GOALS_ACTIVE' and self.rear_dock_goal_done and self.front_dock_goal_done:
            self.get_logger().info("✅ 프론트/리어봇이 카트 결합 준비 위치에 모두 도착했습니다.")
            self.robot_state = 'IDLE'

    def get_current_progress_on_path(self):
        """현재 로봇 위치를 patrol_path에 투영한 누적 진행거리."""
        fallback_progress = self.get_waypoint_progress(self.current_patrol_waypoint_index - 1)
        robot_xy = self.get_robot_xy_in_map()

        if robot_xy is None:
            return fallback_progress

        projection = self.get_closest_point_on_path(
            robot_xy[0],
            robot_xy[1],
            min_progress=fallback_progress
        )
        return projection['progress']

    def get_robot_xy_in_map(self):
        """TF에서 로봇의 map 좌표를 읽는다. 프레임 이름이 다를 수 있어 후보를 순서대로 확인한다."""
        robot_pose = self.get_robot_pose_in_map(('base_link', 'base_footprint', 'rear_base_link'))
        if robot_pose is not None:
            return (robot_pose[0], robot_pose[1])

        self.get_logger().warn("로봇 base TF를 찾지 못해 waypoint feedback 기준으로 진행도를 추정합니다.")
        return None

    def get_robot_pose_in_map(self, frame_candidates):
        """후보 frame 중 하나를 map 좌표계의 (x, y, yaw)로 변환."""
        for frame_id in frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform('map', frame_id, Time())
                translation = transform.transform.translation
                yaw = self.yaw_from_quaternion(transform.transform.rotation)
                return (translation.x, translation.y, yaw)
            except tf2_ros.TransformException:
                continue

        return None

    def get_closest_point_on_path(self, target_x, target_y, min_progress=0.0):
        """경로 중 아직 지나가지 않은 구간에서 타겟과 가장 가까운 점을 찾음."""
        min_dist = float('inf')
        best_projection = None
        cumulative = self.get_path_cumulative_lengths()

        for i in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[i]
            bx, by = self.patrol_path[i+1]

            ab_dx, ab_dy = bx - ax, by - ay
            ab_len_sq = ab_dx**2 + ab_dy**2
            ab_len = math.sqrt(ab_len_sq)

            if ab_len_sq == 0:
                continue

            t = max(0, min(1, ((target_x - ax) * ab_dx + (target_y - ay) * ab_dy) / ab_len_sq))
            progress = cumulative[i] + (t * ab_len)

            if progress + 1e-6 < min_progress:
                continue

            closest_x = ax + t * ab_dx
            closest_y = ay + t * ab_dy

            dist = math.hypot(target_x - closest_x, target_y - closest_y)
            
            if dist < min_dist:
                min_dist = dist
                best_projection = {
                    'point': (closest_x, closest_y),
                    'segment_index': i,
                    't': t,
                    'progress': progress,
                    'distance': dist
                }

        if best_projection is not None:
            return best_projection

        last_x, last_y = self.patrol_path[-1]
        return {
            'point': (last_x, last_y),
            'segment_index': max(0, len(self.patrol_path) - 2),
            't': 1.0,
            'progress': cumulative[-1],
            'distance': math.hypot(target_x - last_x, target_y - last_y)
        }

    def build_waypoint_route_to_exit(self, exit_projection, current_progress):
        """현재 진행도부터 이탈점까지 기존 waypoint를 보존한 route point 목록."""
        exit_progress = exit_projection['progress']
        exit_point = exit_projection['point']
        cumulative = self.get_path_cumulative_lengths()
        route_points = []

        for i, waypoint in enumerate(self.patrol_path):
            waypoint_progress = cumulative[i]
            if current_progress + 0.05 < waypoint_progress < exit_progress - 0.05:
                route_points.append(waypoint)

        if not route_points or math.hypot(route_points[-1][0] - exit_point[0], route_points[-1][1] - exit_point[1]) > 0.05:
            route_points.append(exit_point)

        return route_points

    def get_path_cumulative_lengths(self):
        cumulative = [0.0]
        for i in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[i]
            bx, by = self.patrol_path[i+1]
            cumulative.append(cumulative[-1] + math.hypot(bx - ax, by - ay))
        return cumulative

    def get_waypoint_progress(self, waypoint_index):
        if waypoint_index < 0:
            return 0.0

        cumulative = self.get_path_cumulative_lengths()
        waypoint_index = min(waypoint_index, len(cumulative) - 1)
        return cumulative[waypoint_index]

    def _create_path_poses(self, points, final_yaw=None):
        now_msg = self.get_clock().now().to_msg()
        poses = []

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now_msg
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            if i < len(points) - 1:
                next_x, next_y = points[i+1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif final_yaw is not None:
                yaw = final_yaw
            elif i > 0:
                prev_x, prev_y = points[i-1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)

        return poses

    def _create_pose_stamped(self, x, y, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

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

    # ----------------------------------------------------------
    # 🎯 Nav2 주행 전송 및 체이닝 콜백 함수들
    # ----------------------------------------------------------
    def _send_nav_goal(self, msg: PoseStamped):
        """단일 주행명령 전송 및 BT 스위칭 내부 함수"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        # 🚨 [BT 스위칭 핵심 로직] 3가지 주행 모델 선택
        if not self.is_attached:
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info("🌳 [자동 선택] 디퍼런셜 트리를 사용합니다.")
        elif self.cart_count >= 1:
            goal_msg.behavior_tree = self.ackermann_cart2_bt_path
            self.get_logger().info("🌳 [자동 선택] 아커만(카트 1개 이상) 트리를 사용합니다.")
        else:
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info("🌳 [자동 선택] 아커만(직결/기본) 트리를 사용합니다.")

        try:
            self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
        except Exception as ex:
            self.get_logger().error(f"Nav2 목표 전송 중 예외 발생: {ex}")
            return False

        return True

    def _send_front_nav_goal(self, msg: PoseStamped):
        if not self.front_nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("프론트 Nav2 서버(/front/navigate_to_pose)가 응답하지 않습니다!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        try:
            self.send_front_goal_future = self.front_nav_client.send_goal_async(goal_msg)
            self.send_front_goal_future.add_done_callback(self.front_nav_goal_response_callback)
        except Exception as ex:
            self.get_logger().error(f"프론트 Nav2 목표 전송 중 예외 발생: {ex}")
            return False

        return True

    def nav_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"Nav2 목표 응답 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            return

        if goal_handle is None:
            self.get_logger().error("Nav2 목표 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            return

        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 주행을 거부했습니다.")
            self.robot_state = 'IDLE'
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)

    def front_nav_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"프론트 Nav2 목표 응답 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            return

        if goal_handle is None:
            self.get_logger().error("프론트 Nav2 목표 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            return

        if not goal_handle.accepted:
            self.get_logger().error("프론트 Nav2가 주행을 거부했습니다.")
            self.robot_state = 'IDLE'
            return

        self._get_front_result_future = goal_handle.get_result_async()
        self._get_front_result_future.add_done_callback(self.front_nav_result_callback)

    def front_nav_result_callback(self, future):
        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f"프론트 Nav2 결과 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            return

        if result is None:
            self.get_logger().error("프론트 Nav2 결과 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"프론트 Nav2 주행이 성공하지 못했습니다. status={result.status}")
            self.robot_state = 'IDLE'
            return

        if self.robot_state == 'FRONT_CLEARING':
            self.get_logger().info("✅ 프론트봇 50cm 전진 완료.")
            self.start_rear_heading_alignment()
        elif self.robot_state == 'DOCK_GOALS_ACTIVE':
            self.front_dock_goal_done = True
            self.get_logger().info("✅ 프론트봇이 카트 전방 결합 준비 위치에 도착했습니다.")
            self.check_dock_goal_completion()

    def nav_result_callback(self, future):
        """단일 주행 완료에 따른 상태별 체이닝(연속 실행) 처리"""
        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f"Nav2 결과 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            return

        if result is None:
            self.get_logger().error("Nav2 결과 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Nav2 주행이 성공하지 못했습니다. status={result.status}")
            self.robot_state = 'IDLE'
            return

        if self.robot_state == 'APPROACH_EXIT':
            self.get_logger().info("✅ 이탈점(Exit Point) 도착 완료! 안전 구역을 벗어나 카트로 직진합니다.")
            self.robot_state = 'APPROACH_CART'
            if not self._send_nav_goal(self.cart_final_goal_pose):
                self.robot_state = 'IDLE'
        elif self.robot_state == 'REAR_ALIGNING':
            self.get_logger().info("✅ 리어봇 heading 정렬 완료. 정밀 카트 pose 입력을 기다립니다.")
            self.robot_state = 'WAIT_PRECISE_CART_POSE'
        elif self.robot_state == 'DOCK_GOALS_ACTIVE':
            self.rear_dock_goal_done = True
            self.get_logger().info("✅ 리어봇이 카트 후방 결합 준비 위치에 도착했습니다.")
            self.check_dock_goal_completion()
            
        elif self.robot_state == 'APPROACH_CART':
            self.get_logger().info("🎯 카트 정면 도착 완료! 도킹 대기 모드로 전환합니다.")
            self.robot_state = 'IDLE'
            
        else:
            self.get_logger().info("✅ 일반 목적지에 무사히 도착했습니다!")
            self.robot_state = 'IDLE'

    # --- Waypoint 액션 콜백 ---
    def _send_waypoint_goal(self, waypoints, goal_type):
        if not waypoints:
            self.get_logger().error("Waypoint 목표가 비어 있어 주행을 시작할 수 없습니다.")
            return False

        if not self.wp_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 waypoint 서버가 응답하지 않습니다!")
            return False

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.active_wp_goal_seq += 1
        goal_seq = self.active_wp_goal_seq
        self.active_wp_goal_type = goal_type

        try:
            self.send_wp_future = self.wp_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback_msg, seq=goal_seq, wp_type=goal_type:
                    self.wp_feedback_callback(feedback_msg, seq, wp_type)
            )
            self.send_wp_future.add_done_callback(
                lambda future, seq=goal_seq, wp_type=goal_type:
                    self.wp_goal_response_callback(future, seq, wp_type)
            )
        except Exception as ex:
            self.get_logger().error(f"Waypoint 목표 전송 중 예외 발생: {ex}")
            self.active_wp_goal_type = None
            return False

        return True

    def _cancel_active_waypoint_goal(self):
        self.active_wp_goal_seq += 1

        if self.active_wp_goal_handle is None:
            self.active_wp_goal_type = None
            return

        self.active_wp_goal_handle.cancel_goal_async()
        self.active_wp_goal_handle = None
        self.active_wp_goal_type = None

    def wp_feedback_callback(self, feedback_msg, goal_seq, goal_type):
        if goal_seq != self.active_wp_goal_seq or goal_type != 'PATROL':
            return

        self.current_patrol_waypoint_index = feedback_msg.feedback.current_waypoint

    def wp_goal_response_callback(self, future, goal_seq, goal_type):
        if goal_seq != self.active_wp_goal_seq:
            return

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"Waypoint 목표 응답 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            self.active_wp_goal_type = None
            return

        if goal_handle is None:
            self.get_logger().error("Waypoint 목표 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            self.active_wp_goal_type = None
            return

        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 waypoint 주행을 거부했습니다.")
            self.robot_state = 'IDLE'
            self.active_wp_goal_type = None
            return

        self.active_wp_goal_handle = goal_handle
        self._get_wp_result_future = goal_handle.get_result_async()
        self._get_wp_result_future.add_done_callback(
            lambda future, seq=goal_seq, wp_type=goal_type:
                self.wp_result_callback(future, seq, wp_type)
        )

    def wp_result_callback(self, future, goal_seq, goal_type):
        if goal_seq != self.active_wp_goal_seq:
            return

        self.active_wp_goal_handle = None
        self.active_wp_goal_type = None

        try:
            result = future.result()
        except Exception as ex:
            self.get_logger().error(f"Waypoint 결과 처리 중 예외 발생: {ex}")
            self.robot_state = 'IDLE'
            return

        if result is None or result.result is None:
            self.get_logger().error("Waypoint 결과 응답이 비어 있습니다.")
            self.robot_state = 'IDLE'
            return

        missed_waypoints = list(result.result.missed_waypoints)
        if result.status != GoalStatus.STATUS_SUCCEEDED or missed_waypoints:
            self.get_logger().warn(
                f"Waypoint 주행이 완료되지 않았습니다. status={result.status}, missed={missed_waypoints}"
            )
            self.robot_state = 'IDLE'
            return

        if goal_type == 'PATROL' and self.robot_state == 'PATROL':
            self.get_logger().info("🏁 모든 순찰 경로를 탐색 완료했습니다.")
            self.robot_state = 'IDLE'
        elif goal_type == 'EXIT' and self.robot_state == 'APPROACH_EXIT':
            self.get_logger().info("✅ 경로상 이탈점 도착 완료! 분리 및 결합 준비 시퀀스를 시작합니다.")
            self.start_detach_sequence()

    # ----------------------------------------------------------
    # 🛠️ 내부 헬퍼 메서드 (직접 파라미터 전송 통신용)
    # ----------------------------------------------------------
    def _send_parameters_to_node(self, node_name, param_dict):
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"⚠️ {node_name} 서비스 연결 지연 중.")
            return

        request = SetParameters.Request()
        for name, value in param_dict.items():
            try:
                request.parameters.append(Parameter(name, value=value).to_parameter_msg())
            except Exception as e:
                self.get_logger().error(f"❌ 파라미터 변환 실패 ({name}): {e}")
                
        future = client.call_async(request)
        future.add_done_callback(lambda fut, n=node_name: self._parameter_set_callback(fut, n))

    def _parameter_set_callback(self, future: Future, node_name: str):
        try:
            response = future.result()
            failed = [res.reason for res in response.results if not res.successful]
            if failed:
                self.get_logger().warn(f"⚠️ {node_name} 파라미터 업데이트 거부됨: {failed}")
            else:
                self.get_logger().info(f"✅ {node_name} 파라미터 실시간 적용 완료!")
        except Exception as e:
            self.get_logger().error(f"❌ {node_name} 통신 에러 발생: {e}")

    def _create_polygon(self, front_x, rear_x, width) -> Polygon:
        poly = Polygon()
        poly.points = [
            Point32(x=float(front_x), y=float(-width), z=0.0),
            Point32(x=float(front_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(-width), z=0.0),
        ]
        return poly


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
