#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import os
from ament_index_python.packages import get_package_share_directory

class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        
        # Nav2의 NavigateToPose 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # BT XML 파일이 있는 패키지 경로 찾기 (본인 패키지 이름으로 수정 필요!)
        pkg_dir = get_package_share_directory('cap_sim_2026')
        self.diff_bt_path = os.path.join(pkg_dir, 'bt_xml', 'diff_nav_tree.xml')
        self.ackermann_bt_path = os.path.join(pkg_dir, 'bt_xml', 'ackermann_nav_tree.xml')

    def send_goal(self, x, y, mode="diff"):
        self.get_logger().info("Nav2 액션 서버 기다리는 중...")
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()

        # 1. 목적지 세팅
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0 # 일단 직진 방향 바라보게 세팅
        goal_msg.pose = pose

        # 2. 🚨 모드에 따라 BT XML 갈아끼우기!
        if mode == "diff":
            goal_msg.behavior_tree = self.diff_bt_path
            self.get_logger().info(f"✂️ [디퍼런셜 모드] {x}, {y} 좌표로 출발!")
        elif mode == "ackermann":
            goal_msg.behavior_tree = self.ackermann_bt_path
            self.get_logger().info(f"🚂 [아커만 모드] {x}, {y} 좌표로 출발!")
        else:
            self.get_logger().error("잘못된 모드입니다. 'diff' 또는 'ackermann'을 입력하세요.")
            return

        # 3. 출발 명령 전송
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2가 주행을 거부했습니다 (경로 생성 실패 등)")
            return

        self.get_logger().info("Nav2 주행 시작됨! 결과를 기다립니다...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("✅ 목적지 도착 완료!")
        # 테스트 끝났으니 노드 종료
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Commander()
    
    # ========================================================
    # 🎯 여기서 원하는 좌표와 모드를 선택해서 테스트하세요!
    # 모드: "diff" (분리 시) / "ackermann" (합체 시)
    # ========================================================
    target_x = 5.0
    target_y = 0.0
    target_mode = "ackermann"  
    
    node.send_goal(target_x, target_y, target_mode)
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()
