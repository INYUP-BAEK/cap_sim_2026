#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from std_msgs.msg import Bool, Int32
import os
from ament_index_python.packages import get_package_share_directory

class AutoNavCommander(Node):
    def __init__(self):
        super().__init__('auto_nav_commander')
        
        # 1. Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. BT XML 경로 설정 (본인 패키지명 확인 필수!)
        pkg_dir = get_package_share_directory('cap_sim_2026')
        self.diff_bt_path = os.path.join(pkg_dir, 'bt_xml', 'diff_nav_tree.xml')
        self.ackermann_bt_path = os.path.join(pkg_dir, 'bt_xml', 'ackermann_nav_tree.xml')

        # 3. 로봇의 현재 상태 관리 변수 (초기값: 합체 상태라고 가정)
        self.is_attached = True
        self.cart_count = 0    


        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

        
        # ==========================================================
        # 📡 4. 구독(Subscriber) 설정
        # ==========================================================
        # MuJoCo 브릿지와 동일한 분리/합체 트리거 토픽을 구독하여 스스로 상태를 업데이트합니다.
        self.docking_sub = self.create_subscription(Bool, '/docking_state', self.docking_callback, 10)

        # 목적지를 받을 커스텀 토픽 (터미널이나 상위 노드에서 여기로 목적지를 쏩니다)
        self.goal_sub = self.create_subscription(PoseStamped, '/mission_goal', self.mission_goal_callback, 10)

        self.get_logger().info("🤖 [Auto Commander] 실행 완료! 임무 좌표와 트리거를 기다립니다.")

    # ----------------------------------------------------------
    # 🔄 상태 업데이트 콜백 함수들
    # ----------------------------------------------------------
    def docking_callback(self, msg):
        # msg.data가 True면 분리(디퍼런셜), False면 합체(아커만)
        self.is_attached = msg.data
        self.update_nav2_footprint()
        if self.is_attached :
            mode_str = "아커만(합체)"
        else:
            mode_str = "디퍼런셜(분리)"
        self.get_logger().info(f"🔄 [상태 변경 감지] 현재 모드: {mode_str}")

    def update_nav2_footprint(self):
        msg = Polygon()
        rear_bumper_x, width = -0.3, 0.25

        if self.is_attached:
            current_wheelbase = 0.48 if self.cart_count == 0 else 1.55 + (self.cart_count - 1) * 0.85
            front_bumper_x = current_wheelbase + 0.3
        else:
            front_bumper_x = 0.3

        msg.points = [
            Point32(x=front_bumper_x, y=-width, z=0.0), 
            Point32(x=front_bumper_x, y=width,  z=0.0), 
            Point32(x=rear_bumper_x,  y=width,  z=0.0), 
            Point32(x=rear_bumper_x,  y=-width, z=0.0)  
        ]
        self.global_footprint_pub.publish(msg)
        self.local_footprint_pub.publish(msg)

    # ----------------------------------------------------------
    # 🎯 목적지 수신 및 Nav2 자동 실행 콜백 함수
    # ----------------------------------------------------------
    def mission_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"📥 새로운 목적지 수신: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 서버가 응답하지 않습니다!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # 🚨 [핵심 로직] 현재 로봇 상태(is_attached)에 따라 트리를 자동으로 결정!
        if self.is_attached:
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info("🚂 [자동 선택] 아커만 트리를 사용하여 주행을 시작합니다.")
        else:
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info("✂️ [자동 선택] 디퍼런셜 트리를 사용하여 주행을 시작합니다.")

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

if __name__ == '__main__':
    main()