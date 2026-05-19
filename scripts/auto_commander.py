#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32, PointStamped
from std_msgs.msg import Bool, Int32
import os
import subprocess
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

        self.front_global_footprint_pub = self.create_publisher(Polygon, '/front/global_costmap/footprint', 10)
        self.front_local_footprint_pub = self.create_publisher(Polygon, '/front/local_costmap/footprint', 10)

        # ==========================================================
        # ⏱️ 3초 쿨다운 관리를 위한 변수 초기화
        # ==========================================================
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
        self.cart_target_sub = self.create_subscription(
            PointStamped, "/vision/cart_target_ground", self.cart_target_callback, 10
        )

        self.get_logger().info(
            "🤖 [Auto Commander] 실행 완료! 임무 좌표와 트리거를 기다립니다."
        )

    # ----------------------------------------------------------
    # 🔄 상태 업데이트 콜백 함수들
    # ----------------------------------------------------------
    def load_nav2_yaml_params(self, is_long_wheelbase):
        # 🚨 주의: 시스템 환경에 맞는 절대 경로 사용
        if is_long_wheelbase:
            yaml_path = "/home/baek/colcon_ws/src/cap_sim_2026/config/nav2_real_acman_params.yaml"
            self.get_logger().info("🚛 [파라미터 체인지] 카트 결합 모드(1.45m) YAML 적용 중...")
        else:
            yaml_path = "/home/baek/colcon_ws/src/cap_sim_2026/config/nav2_real_acman_params_noncart.yaml"
            self.get_logger().info("🏎️ [파라미터 체인지] 직결 모드(0.48m) YAML 적용 중...")

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"❌ YAML 파일을 찾을 수 없습니다: {yaml_path}")
            return

        # 🚨 [최적화 반영] 변경이 필요한 핵심 노드 리스트 (Costmap 추가)
        target_nodes = [
            '/controller_server',
            '/planner_server',
            '/velocity_smoother',
            '/local_costmap/local_costmap',
            '/global_costmap/global_costmap'
        ]

        # 비동기로 터미널 명령어 실행 (Main Thread 블로킹 방지)
        # for node in target_nodes:
        #     try:
        #         subprocess.Popen(
        #             ['ros2', 'param', 'load', node, yaml_path],
        #             stdout=subprocess.DEVNULL,
        #             stderr=subprocess.DEVNULL
        #         )
        #     except Exception as e:
        #         self.get_logger().error(f"❌ {node} 파라미터 로드 실패: {e}")
        
        self.get_logger().info("✅ 파라미터 실시간 덮어씌우기 명령 전달 완료!")
    
    def docking_callback(self, msg):
        self.is_attached = msg.data
        
        # 1. 동적 풋프린트 실시간 퍼블리시 (항상 최우선 적용됨)
        self.update_nav2_footprint()

        # 2. 결합 상태에 맞춰 YAML 파라미터 파일 전체 스위칭
        is_long = self.is_attached and (self.cart_count > 0)
        # self.load_nav2_yaml_params(is_long_wheelbase=is_long)
        
        if self.is_attached:
            mode_str = "아커만(합체) - 프론트봇 풋프린트 최소화 적용"
        else:
            mode_str = "디퍼런셜(분리) - 개별 풋프린트 복구"
        self.get_logger().info(f"🔄 [상태 변경 감지] 현재 모드: {mode_str}")

    def update_nav2_footprint(self):
        rear_msg = Polygon()
        front_msg = Polygon()

        rear_width = 0.25
        rear_bumper_x = -0.3

        if self.is_attached:
            current_wheelbase = (
                    1.45 if self.cart_count == 0 else 1.55 + (self.cart_count - 1) * 0.85
            )
            rear_front_bumper_x = current_wheelbase + 0.3

            rear_msg.points = [
                Point32(x=rear_front_bumper_x, y=-rear_width, z=0.0),
                Point32(x=rear_front_bumper_x, y=rear_width, z=0.0),
                Point32(x=rear_bumper_x, y=rear_width, z=0.0),
                Point32(x=rear_bumper_x, y=-rear_width, z=0.0),
            ]

            tiny_size = 0.01
            front_msg.points = [
                Point32(x=tiny_size, y=-tiny_size, z=0.0),
                Point32(x=tiny_size, y=tiny_size, z=0.0),
                Point32(x=-tiny_size, y=tiny_size, z=0.0),
                Point32(x=-tiny_size, y=-tiny_size, z=0.0),
            ]

        else:
            rear_front_bumper_x = 0.3
            rear_msg.points = [
                Point32(x=rear_front_bumper_x, y=-rear_width, z=0.0),
                Point32(x=rear_front_bumper_x, y=rear_width, z=0.0),
                Point32(x=rear_bumper_x, y=rear_width, z=0.0),
                Point32(x=rear_bumper_x, y=-rear_width, z=0.0),
            ]

            front_front_bumper_x = 0.3
            front_rear_bumper_x = -0.3
            front_width = 0.25

            front_msg.points = [
                Point32(x=front_front_bumper_x, y=-front_width, z=0.0),
                Point32(x=front_front_bumper_x, y=front_width, z=0.0),
                Point32(x=front_rear_bumper_x, y=front_width, z=0.0),
                Point32(x=front_rear_bumper_x, y=-front_width, z=0.0),
            ]

        self.global_footprint_pub.publish(rear_msg)
        self.local_footprint_pub.publish(rear_msg)

        self.front_global_footprint_pub.publish(front_msg)
        self.front_local_footprint_pub.publish(front_msg)

    # ----------------------------------------------------------
    # 🛒 카트 타겟 수신 및 좌표 변환 콜백 함수
    # ----------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        current_time = self.get_clock().now()

        if self.last_cart_goal_time is not None:
            time_diff = current_time - self.last_cart_goal_time
            if time_diff.nanoseconds < 5e8:
                return

        cart_x = msg.point.x
        cart_y = msg.point.y

        distance_to_cart = math.sqrt(cart_x**2 + cart_y**2)
        yaw_to_cart = math.atan2(cart_y, cart_x)

        stop_distance = 1.5 if self.is_attached else 1.0

        if distance_to_cart <= stop_distance:
            self.get_logger().info("✅ 카트가 이미 목표 안전 거리 이내에 있습니다.")
            return

        ratio = (distance_to_cart - stop_distance) / distance_to_cart
        local_goal_x = cart_x * ratio
        local_goal_y = cart_y * ratio

        local_pose = PoseStamped()
        local_pose.header.frame_id = msg.header.frame_id
        local_pose.header.stamp = current_time.to_msg()

        local_pose.pose.position.x = local_goal_x
        local_pose.pose.position.y = local_goal_y
        local_pose.pose.position.z = 0.0

        local_pose.pose.orientation.x = 0.0
        local_pose.pose.orientation.y = 0.0
        local_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',                  
                msg.header.frame_id,    
                rclpy.time.Time()
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)

            global_goal_pose = PoseStamped()
            global_goal_pose.header.frame_id = 'map'
            global_goal_pose.header.stamp = current_time.to_msg()
            global_goal_pose.pose = transformed_pose

            self.get_logger().info(
                f"🛒 [시선 일치] 카트 앞 목표 좌표: X={global_goal_pose.pose.position.x:.2f}, Y={global_goal_pose.pose.position.y:.2f}"
            )

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