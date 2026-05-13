#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from std_msgs.msg import Bool
import os
from ament_index_python.packages import get_package_share_directory


class MultiBotCommander(Node):
    def __init__(self):
        super().__init__("multi_bot_commander")

        # ==========================================================
        # 1. 듀얼 Nav2 액션 클라이언트 (네임스페이스 재정비)
        # ==========================================================
        # 리어봇/아커만 모드는 네임스페이스 없음 (Root)
        self.rear_nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 프론트봇은 /front 네임스페이스 사용
        self.front_nav_client = ActionClient(
            self, NavigateToPose, "/front/navigate_to_pose"
        )

        # ==========================================================
        # 2. BT XML 경로 설정
        # ==========================================================
        pkg_dir = get_package_share_directory("cap_sim_2026")
        self.diff_bt_path = os.path.join(pkg_dir, "bt_xml", "diff_nav_tree.xml")
        self.ackermann_bt_path = os.path.join(
            pkg_dir, "bt_xml", "ackermann_nav_tree.xml"
        )

        # ==========================================================
        # 3. 듀얼 풋프린트 퍼블리셔 (네임스페이스 재정비)
        # ==========================================================
        # 리어봇 코스트맵 (네임스페이스 없음)
        self.rear_global_fp_pub = self.create_publisher(
            Polygon, "global_costmap/footprint", 10
        )
        self.rear_local_fp_pub = self.create_publisher(
            Polygon, "local_costmap/footprint", 10
        )

        # 프론트봇 코스트맵 (/front)
        self.front_global_fp_pub = self.create_publisher(
            Polygon, "/front/global_costmap/footprint", 10
        )
        self.front_local_fp_pub = self.create_publisher(
            Polygon, "/front/local_costmap/footprint", 10
        )

        # 상태 변수
        self.is_attached = True  # 초기값: 합체 상태
        self.cart_count = 0

        # ==========================================================
        # 4. 구독(Subscriber) 설정
        # ==========================================================
        self.docking_sub = self.create_subscription(
            Bool, "docking_state", self.docking_callback, 10
        )

        # 리어봇(마스터) 목적지: /mission_goal
        self.rear_goal_sub = self.create_subscription(
            PoseStamped, "mission_goal", self.rear_goal_callback, 10
        )
        # 프론트봇 단독 목적지: /front/mission_goal
        self.front_goal_sub = self.create_subscription(
            PoseStamped, "/front/mission_goal", self.front_goal_callback, 10
        )

        self.get_logger().info(
            "🤖 [DDS Multi-Bot Commander] 가동 완료! 메인(Rear) / 서브(Front) 통신 대기 중."
        )
        self.update_dual_footprints()

    def docking_callback(self, msg):
        previous_state = self.is_attached
        self.is_attached = msg.data

        if self.is_attached != previous_state:
            mode_str = (
                "아커만 (합체, 리어봇 주도)"
                if self.is_attached
                else "디퍼런셜 (분리, 독립 주행)"
            )
            self.get_logger().info(f"🔄 [형태 변환] 모드 전환: {mode_str}")

            if self.is_attached:
                self.cancel_front_bot_goal()

            self.update_dual_footprints()

    def update_dual_footprints(self):
        rear_fp_msg = Polygon()
        front_fp_msg = Polygon()
        width = 0.25
        rear_bumper_x = -0.3

        if self.is_attached:
            # [합체] 리어봇이 프론트봇 크기까지 덮음
            current_wheelbase = (
                0.48 if self.cart_count == 0 else 1.55 + (self.cart_count - 1) * 0.85
            )
            huge_front_x = current_wheelbase + 0.3

            rear_fp_msg.points = [
                Point32(x=huge_front_x, y=-width, z=0.0),
                Point32(x=huge_front_x, y=width, z=0.0),
                Point32(x=rear_bumper_x, y=width, z=0.0),
                Point32(x=rear_bumper_x, y=-width, z=0.0),
            ]
            # 프론트봇은 점(Point) 처리하여 잉여 연산 방지
            front_fp_msg.points = [
                Point32(x=0.01, y=-0.01, z=0.0),
                Point32(x=0.01, y=0.01, z=0.0),
                Point32(x=-0.01, y=0.01, z=0.0),
                Point32(x=-0.01, y=-0.01, z=0.0),
            ]
        else:
            # [분리] 각자 크기 복구
            small_front_x = 0.3
            rear_fp_msg.points = [
                Point32(x=small_front_x, y=-width, z=0.0),
                Point32(x=small_front_x, y=width, z=0.0),
                Point32(x=rear_bumper_x, y=width, z=0.0),
                Point32(x=rear_bumper_x, y=-width, z=0.0),
            ]
            front_fp_msg.points = rear_fp_msg.points

        self.rear_global_fp_pub.publish(rear_fp_msg)
        self.rear_local_fp_pub.publish(rear_fp_msg)
        self.front_global_fp_pub.publish(front_fp_msg)
        self.front_local_fp_pub.publish(front_fp_msg)
        self.get_logger().info("📐 풋프린트 동기화 완료.")

    def rear_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"📥 [Rear/Ackermann] 명령 수신")
        if not self.rear_nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Rear Nav2 서버 다운!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        goal_msg.behavior_tree = (
            self.ackermann_bt_path if self.is_attached else self.diff_bt_path
        )
        self.rear_nav_client.send_goal_async(goal_msg)

    def front_goal_callback(self, msg: PoseStamped):
        if self.is_attached:
            self.get_logger().warn("⚠️ [Front Bot] 합체 상태! 단독 주행 명령 무시.")
            return

        self.get_logger().info(f"📥 [Front Bot] 독립 명령 수신")
        if not self.front_nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Front Nav2 서버 다운!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        goal_msg.behavior_tree = self.diff_bt_path
        self.front_nav_client.send_goal_async(goal_msg)

    def cancel_front_bot_goal(self):
        self.get_logger().info("🛑 프론트봇 독립 주행 강제 정지 명령 전송 (구현 필요)")


def main(args=None):
    rclpy.init(args=args)
    node = MultiBotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
