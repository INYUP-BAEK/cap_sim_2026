#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import SetParameters

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32, PointStamped
from std_msgs.msg import Bool, UInt16

from ament_index_python.packages import get_package_share_directory
import tf2_ros
import tf2_geometry_msgs


class AutoNavCommander(Node):
    def __init__(self):
        super().__init__("auto_nav_commander")

        # 1. Nav2 액션 클라이언트 및 BT XML 3가지 경로 설정
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        pkg_dir = get_package_share_directory("cap_sim_2026")
        
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_nav_tree.xml")
        self.ackermann_cart2_bt_path = os.path.join(pkg_dir, "bt_xml", "ackermann_cart2_nav_tree.xml")

        # 2. 로봇의 현재 상태 관리 변수
        self.is_attached = True
        self.cart_count = 0
        self.last_cart_goal_time = None

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

        self.get_logger().info("🤖 [Auto Commander] 실행 완료! 임무 좌표와 트리거를 기다립니다.")

    # ----------------------------------------------------------
    # 🔄 상태 업데이트 콜백 함수들
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

        # 2. YAML 파일을 통째로 읽는 대신, 필요한 파라미터만 직접 딕셔너리로 쏴줍니다.
        if self.is_attached:
            if self.cart_count >= 1:
                # 🚛 [합체 - 카트 2개 이상] 무겁고 둔함
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
            
            # 합체 모드 공통: 시야가 가려진 리어 카메라는 끄고 프론트 카메라를 켭니다.
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
            # 분리 모드: 자체 리어 카메라는 켜고, 프론트 카메라는 끕니다.
            camera_params = {
                'stvl_camera_rear_layer.enabled': True,
                'stvl_camera_front_layer.enabled': False
            }
            mode_str = "디퍼런셜(분리) - 리어 카메라 활성화"

        # 코스트맵과 벨로시티 스무더에 파라미터 직접 전송
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
    # 🎯 목적지 수신 및 Nav2 자동 실행 콜백 함수 (BT 스위칭)
    # ----------------------------------------------------------
    def mission_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"📥 새로운 목적지 수신: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # 🚨 [BT 스위칭 핵심 로직] 상태와 카트 개수에 맞춰 트리를 고릅니다.
        if not self.is_attached:
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info("✂️ [자동 선택] 디퍼런셜 트리를 사용합니다.")
        elif self.cart_count >= 1:
            goal_msg.behavior_tree = self.ackermann_cart2_bt_path
            self.get_logger().info("🚛 [자동 선택] 아커만(카트 2개) 트리를 사용합니다.")
        else:
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info("🏎️ [자동 선택] 아커만(직결/기본) 트리를 사용합니다.")

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


    # ----------------------------------------------------------
    # 🛒 카트 타겟 수신 및 좌표 변환 콜백 함수
    # ----------------------------------------------------------
    def cart_target_callback(self, msg: PointStamped):
        current_time = self.get_clock().now()

        if self.last_cart_goal_time is not None:
            if (current_time - self.last_cart_goal_time).nanoseconds < 5e8:
                return

        cart_x, cart_y = msg.point.x, msg.point.y
        distance_to_cart = math.hypot(cart_x, cart_y)
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
        local_pose.pose.orientation.z = math.sin(yaw_to_cart / 2.0)
        local_pose.pose.orientation.w = math.cos(yaw_to_cart / 2.0)

        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)

            global_goal_pose = PoseStamped()
            global_goal_pose.header.frame_id = 'map'
            global_goal_pose.header.stamp = current_time.to_msg()
            global_goal_pose.pose = transformed_pose

            self.get_logger().info(f"🛒 [시선 일치] 카트 앞 목표 좌표: X={global_goal_pose.pose.position.x:.2f}")

            self.mission_goal_callback(global_goal_pose)
            self.last_cart_goal_time = current_time

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"로컬라이제이션 대기 중 또는 TF 변환 실패: {ex}")


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