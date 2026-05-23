#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import SetParameters

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, Polygon, Point32, PointStamped
from std_msgs.msg import Bool, UInt16, Empty

from ament_index_python.packages import get_package_share_directory
import tf2_ros
import tf2_geometry_msgs


class AutoNavCommander(Node):
    def __init__(self):
        super().__init__("auto_nav_commander")

        # 1. Nav2 액션 클라이언트 및 BT XML 경로 설정 (3가지 트리 적용)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.wp_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        
        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_nav_tree.xml")
        self.ackermann_cart2_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_cart2_nav_tree.xml")

        # 2. 로봇 상태 및 주행 관리 변수 (State Machine)
        self.is_attached = True
        self.cart_count = 0
        
        self.robot_state = 'IDLE'           # IDLE, PATROL, APPROACH_EXIT, APPROACH_CART
        self.active_wp_goal_handle = None   # 순찰 취소용 핸들
        self.cart_final_goal_pose = None    # 카트 최종 목적지 백업용
        
        # 🚨 [수정 필요] 로봇이 순찰할 커스텀 글로벌 경로 (X, Y) 리스트
        self.patrol_path = [
            (6.82, 2.5), (7.72, 2.45), (4.52, 1.95), (4.67, 1.1)]


        # 3. 풋프린트 퍼블리셔 설정
        self.global_footprint_pub = self.create_publisher(Polygon, "/global_costmap/footprint", 10)
        self.local_footprint_pub = self.create_publisher(Polygon, "/local_costmap/footprint", 10)
        self.front_global_footprint_pub = self.create_publisher(Polygon, '/front/global_costmap/footprint', 10)
        self.front_local_footprint_pub = self.create_publisher(Polygon, '/front/local_costmap/footprint', 10)

        # 4. TF2 버퍼 및 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 구독(Subscriber) 설정
        self.docking_sub = self.create_subscription(Bool, "/docking_state", self.docking_callback, 10)
        self.cart_count_sub = self.create_subscription(UInt16, "/cart_count", self.cart_count_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/mission_goal", self.mission_goal_callback, 10)
        self.cart_target_sub = self.create_subscription(PointStamped, "/vision/cart_target_ground", self.cart_target_callback, 10)
        
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
        self.robot_state = 'PATROL'

        waypoints = []
        for i, (x, y) in enumerate(self.patrol_path):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            if i < len(self.patrol_path) - 1:
                next_x, next_y = self.patrol_path[i+1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0
            waypoints.append(pose)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.send_wp_future = self.wp_client.send_goal_async(goal_msg)
        self.send_wp_future.add_done_callback(self.wp_goal_response_callback)

    def mission_goal_callback(self, msg: PoseStamped):
        """단일 일반 목표 수신 시 (수동 제어 등)"""
        self.get_logger().info(f"📥 일반 목적지 수신: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        
        if self.active_wp_goal_handle:
            self.active_wp_goal_handle.cancel_goal_async()
            self.active_wp_goal_handle = None
            
        self.robot_state = 'IDLE' 
        self._send_nav_goal(msg)

    # ----------------------------------------------------------
    # 🛒 카트 타겟 수신 및 이탈점 계산 콜백
    # ----------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        """카트 발견 시 호출되어 이탈점 계산 후 궤도 수정"""
        if self.robot_state != 'PATROL':
            return

        current_time = self.get_clock().now()
        cart_x, cart_y = msg.point.x, msg.point.y
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

        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            global_goal_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)

            target_global_x = global_goal_pose.pose.position.x
            target_global_y = global_goal_pose.pose.position.y

            self.get_logger().info(f"👀 카트 발견! (글로벌 좌표 X={target_global_x:.2f}, Y={target_global_y:.2f})")

            exit_x, exit_y = self.get_closest_point_on_path(target_global_x, target_global_y)

            if self.active_wp_goal_handle is not None:
                self.get_logger().info("🛑 기존 순찰 경로 주행을 중지하고 궤도 이탈을 시작합니다.")
                self.active_wp_goal_handle.cancel_goal_async()
                self.active_wp_goal_handle = None

            self.robot_state = 'APPROACH_EXIT'
            
            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = current_time.to_msg()
            final_pose.pose = global_goal_pose.pose
            self.cart_final_goal_pose = final_pose

            exit_pose = PoseStamped()
            exit_pose.header.frame_id = 'map'
            exit_pose.header.stamp = current_time.to_msg()
            exit_pose.pose.position.x = exit_x
            exit_pose.pose.position.y = exit_y
            
            exit_yaw = math.atan2(target_global_y - exit_y, target_global_x - exit_x)
            exit_pose.pose.orientation.z = math.sin(exit_yaw / 2.0)
            exit_pose.pose.orientation.w = math.cos(exit_yaw / 2.0)

            self.get_logger().info(f"📍 가장 가까운 안전 이탈점(X={exit_x:.2f}, Y={exit_y:.2f})으로 이동합니다.")
            self._send_nav_goal(exit_pose)

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"TF 변환 실패(로컬라이제이션 대기 중): {ex}")

    def get_closest_point_on_path(self, target_x, target_y):
        """경로 리스트를 순회하며 타겟(카트)과 가장 가까운 선분 상의 수선의 발을 찾음"""
        min_dist = float('inf')
        exit_point = (0.0, 0.0)

        for i in range(len(self.patrol_path) - 1):
            ax, ay = self.patrol_path[i]
            bx, by = self.patrol_path[i+1]

            ab_dx, ab_dy = bx - ax, by - ay
            ab_len_sq = ab_dx**2 + ab_dy**2

            if ab_len_sq == 0: continue

            t = max(0, min(1, ((target_x - ax) * ab_dx + (target_y - ay) * ab_dy) / ab_len_sq))
            
            closest_x = ax + t * ab_dx
            closest_y = ay + t * ab_dy

            dist = math.hypot(target_x - closest_x, target_y - closest_y)
            
            if dist < min_dist:
                min_dist = dist
                exit_point = (closest_x, closest_y)

        return exit_point

    # ----------------------------------------------------------
    # 🎯 Nav2 주행 전송 및 체이닝 콜백 함수들
    # ----------------------------------------------------------
    def _send_nav_goal(self, msg: PoseStamped):
        """단일 주행명령 전송 및 BT 스위칭 내부 함수"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return

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

        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 주행을 거부했습니다.")
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """단일 주행 완료에 따른 상태별 체이닝(연속 실행) 처리"""
        if self.robot_state == 'APPROACH_EXIT':
            self.get_logger().info("✅ 이탈점(Exit Point) 도착 완료! 안전 구역을 벗어나 카트로 직진합니다.")
            self.robot_state = 'APPROACH_CART'
            self._send_nav_goal(self.cart_final_goal_pose)
            
        elif self.robot_state == 'APPROACH_CART':
            self.get_logger().info("🎯 카트 정면 도착 완료! 도킹 대기 모드로 전환합니다.")
            self.robot_state = 'IDLE'
            
        else:
            self.get_logger().info("✅ 일반 목적지에 무사히 도착했습니다!")
            self.robot_state = 'IDLE'

    # --- Waypoint 액션 콜백 ---
    def wp_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 순찰(Waypoint)을 거부했습니다.")
            self.robot_state = 'IDLE'
            return
        self.active_wp_goal_handle = goal_handle
        self._get_wp_result_future = goal_handle.get_result_async()
        self._get_wp_result_future.add_done_callback(self.wp_result_callback)

    def wp_result_callback(self, future):
        if self.robot_state == 'PATROL':
            self.get_logger().info("🏁 모든 순찰 경로를 탐색 완료했습니다.")
            self.robot_state = 'IDLE'
            self.active_wp_goal_handle = None

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