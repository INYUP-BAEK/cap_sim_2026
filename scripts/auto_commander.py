#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32, PointStamped
from std_msgs.msg import Bool, Int32
import os
from ament_index_python.packages import get_package_share_directory

# TF 변환을 위한 모듈 임포트
import tf2_ros
import tf2_geometry_msgs
import math

class AutoNavCommander(Node):
    def __init__(self):
        super().__init__("auto_nav_commander")

        # 1. Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 2. BT XML 경로 설정
        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(
            pkg_dir, "bt_xml", "ackermann_nav_tree.xml"
        )

        # 3. 로봇의 현재 상태 관리 변수
        self.is_attached = True
        self.cart_count = 0

        self.global_footprint_pub = self.create_publisher(
            Polygon, "/global_costmap/footprint", 10
        )
        self.local_footprint_pub = self.create_publisher(
            Polygon, "/local_costmap/footprint", 10
        )

        # ==========================================================
        # ⏱️ [추가] 3초 쿨다운 관리를 위한 변수 초기화
        # ==========================================================
        # 0초로 초기화하여 첫 번째 메시지는 바로 처리되도록 함
        self.last_cart_goal_time = None

        # ==========================================================
        # 🎯 4. TF2 버퍼 및 리스너 설정
        # ==========================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==========================================================
        # 📡 5. 구독(Subscriber) 설정
        # ==========================================================
        self.docking_sub = self.create_subscription(
            Bool, "/docking_state", self.docking_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/mission_goal", self.mission_goal_callback, 10
        )

        # 비전 노드로부터 카트의 로컬 좌표(PointStamped)를 받는 토픽 구독
        self.cart_target_sub = self.create_subscription(
            PointStamped, "/vision/cart_target_ground", self.cart_target_callback, 10
        )

        self.get_logger().info(
            "🤖 [Auto Commander] 실행 완료! 임무 좌표와 트리거를 기다립니다."
        )

    # ----------------------------------------------------------
    # 🔄 상태 업데이트 콜백 함수들
    # ----------------------------------------------------------
    def docking_callback(self, msg):
        self.is_attached = msg.data
        self.update_nav2_footprint()
        if self.is_attached:
            mode_str = "아커만(합체)"
        else:
            mode_str = "디퍼런셜(분리)"
        self.get_logger().info(f"🔄 [상태 변경 감지] 현재 모드: {mode_str}")

    def update_nav2_footprint(self):
        msg = Polygon()
        rear_bumper_x, width = -0.3, 0.25

        if self.is_attached:
            current_wheelbase = (
                1.45 if self.cart_count == 0 else 1.55 + (self.cart_count - 1) * 0.85
            )
            front_bumper_x = current_wheelbase + 0.3
        else:
            front_bumper_x = 0.3

        msg.points = [
            Point32(x=front_bumper_x, y=-width, z=0.0),
            Point32(x=front_bumper_x, y=width, z=0.0),
            Point32(x=rear_bumper_x, y=width, z=0.0),
            Point32(x=rear_bumper_x, y=-width, z=0.0),
        ]
        self.global_footprint_pub.publish(msg)
        self.local_footprint_pub.publish(msg)

    # ----------------------------------------------------------
    # 🛒 카트 타겟 수신 및 좌표 변환 콜백 함수 (정지 거리 + 시선 각도 적용)
    # ----------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        current_time = self.get_clock().now()

        # 쿨다운 계산 (3초 제한)
        if self.last_cart_goal_time is not None:
            time_diff = current_time - self.last_cart_goal_time
            if time_diff.nanoseconds < 5e8:
                return

        # ==========================================================
        # 🚨 1. 카트 앞 대기 위치 및 바라보는 각도(Yaw) 계산
        # ==========================================================
        cart_x = msg.point.x
        cart_y = msg.point.y

        # 로봇(원점)에서 카트까지의 직선 거리 및 각도(라디안) 계산
        distance_to_cart = math.sqrt(cart_x**2 + cart_y**2)
        yaw_to_cart = math.atan2(cart_y, cart_x)  # 로봇이 카트를 바라보는 로컬 각도

        # 앞서 말씀드린 상태별 유동적 정지 거리 (예시)
        # 아커만(합체) 상태면 로봇 길이가 기니까 1.5m, 아니면 1.0m
        stop_distance = 1.5 if self.is_attached else 1.0

        if distance_to_cart <= stop_distance:
            self.get_logger().info("✅ 카트가 이미 목표 안전 거리 이내에 있습니다.")
            return

        # 벡터 오프셋을 적용한 로컬 목표 위치
        ratio = (distance_to_cart - stop_distance) / distance_to_cart
        local_goal_x = cart_x * ratio
        local_goal_y = cart_y * ratio

        # ==========================================================
        # 🚨 2. 로컬 PoseStamped 생성 (위치 + 자세)
        # ==========================================================
        local_pose = PoseStamped()
        local_pose.header.frame_id = msg.header.frame_id  # 'base_link' 등
        local_pose.header.stamp = current_time.to_msg()

        local_pose.pose.position.x = local_goal_x
        local_pose.pose.position.y = local_goal_y
        local_pose.pose.position.z = 0.0

        # 2D 평면(Yaw) 회전을 Quaternion(w, x, y, z)으로 변환 (Roll=0, Pitch=0)
        local_pose.pose.orientation.x = 0.0
        local_pose.pose.orientation.y = 0.0
        local_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        try:
            # ==========================================================
            # 🚨 3. 위치와 자세 변환 (수정됨)
            # ==========================================================
            transform = self.tf_buffer.lookup_transform(
                'map',                  
                msg.header.frame_id,    
                rclpy.time.Time()
            )

            # [핵심 수정] PoseStamped 전체가 아니라 .pose 부분만 넘겨줍니다.
            transformed_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)

            # 변환된 순수 Pose를 다시 글로벌 PoseStamped로 감싸기
            global_goal_pose = PoseStamped()
            global_goal_pose.header.frame_id = 'map'
            global_goal_pose.header.stamp = current_time.to_msg()
            global_goal_pose.pose = transformed_pose

            self.get_logger().info(
                f"🛒 [시선 일치] 카트 앞 목표 좌표: X={global_goal_pose.pose.position.x:.2f}, Y={global_goal_pose.pose.position.y:.2f}"
            )

            # 4. 최종 계산된 PoseStamped를 목적지로 전송
            self.mission_goal_callback(global_goal_pose)
            self.last_cart_goal_time = current_time

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"로컬라이제이션 대기 중 또는 TF 변환 실패: {ex}")

    # ----------------------------------------------------------
    # 🎯 목적지 수신 및 Nav2 자동 실행 콜백 함수
    # ----------------------------------------------------------
    def mission_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"📥 새로운 목적지 수신: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}"
        )

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        if self.is_attached:
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info(
                "🚂 [자동 선택] 아커만 트리를 사용하여 주행을 시작합니다."
            )
        else:
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info(
                "✂️ [자동 선택] 디퍼런셜 트리를 사용하여 주행을 시작합니다."
            )

        # Nav2로 목표 전송
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 주행을 거부했습니다.")
            return
        self.get_logger().info("🚀 Nav2 자율주행 시작!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("✅ 목적지에 무사히 도착했습니다!")


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
