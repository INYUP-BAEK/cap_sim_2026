#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def create_pose(navigator, x, y, yaw, frame_id='map'):
    """x, y, yaw(라디안) 값을 Nav2용 PoseStamped 메시지로 변환하는 헬퍼 함수"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    # Yaw 각도를 쿼터니언으로 변환
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def main():
    rclpy.init()

    # 1. 네비게이터 및 통신 노드 초기화
    # 리어봇(마스터)은 기본 네임스페이스, 프론트봇은 'front' 네임스페이스 사용
    rear_nav = BasicNavigator()
    front_nav = BasicNavigator(namespace='front')

    bridge_commander = rclpy.create_node('scenario_commander')
    detach_pub = bridge_commander.create_publisher(Bool, '/detach_trigger', 10)
    attach_pub = bridge_commander.create_publisher(Int32, '/robot_attach_topic', 10)

    # Nav2 서버들이 완전히 켜질 때까지 대기
    rear_nav.waitUntilNav2Active(localizer='bt_navigator')
    front_nav.waitUntilNav2Active(localizer='bt_navigator')
    bridge_commander.get_logger().info("🔥 두 로봇의 Nav2 시스템 준비 완료! 시나리오를 시작합니다.")

    # =====================================================================
    # [Step 1 & 2] 웨이포인트 팔로잉 (카트 탐색 및 접근) - 아커만 모드
    # =====================================================================
    bridge_commander.get_logger().info("▶️ [Step 1&2] 카트 탐색을 위해 웨이포인트 주행을 시작합니다 (아커만 모드)")
    
    # 예시 웨이포인트 3개 (실제 맵에 맞게 수정 필요)
    waypoints = [
        create_pose(rear_nav, 7.2, 2.5, 0.0),
        create_pose(rear_nav, 5.3, -4.7, 3.14),
        create_pose(rear_nav, -6.3, -1.0, -1.57) # 카트와 가장 가까운 최종 도착지
    ]
    
    rear_nav.followWaypoints(waypoints)
    while not rear_nav.isTaskComplete():
        time.sleep(1.0)
        # 필요시 여기서 카메라 객체 인식 코드를 섞을 수 있습니다.
        
    bridge_commander.get_logger().info("✅ 카트 앞 도착 완료!")

    # =====================================================================
    # [Step 3] 로봇 분리 (디퍼런셜 모드 전환)
    # =====================================================================
    bridge_commander.get_logger().info("▶️ [Step 3] 로봇 합체 해제 (디퍼런셜 모드로 전환)")
    
    detach_msg = Bool()
    detach_msg.data = True
    detach_pub.publish(detach_msg)
    time.sleep(2.0) # 물리 엔진이 안정화될 시간 2초 부여

    # =====================================================================
    # [Step 4] 카트 앞뒤 정렬 및 도킹
    # =====================================================================
    bridge_commander.get_logger().info("▶️ [Step 4] 카트 앞뒤로 정렬하기 위해 개별 주행 시작")
    
    # 카트 1번을 기준으로 앞/뒤 도킹 위치 (실제 좌표 측정 필요)
    # 프론트봇은 카트 앞(cart_front)으로, 리어봇은 카트 뒤(cart_rear)로 이동
    front_docking_pose = create_pose(front_nav, 5.0, -2.0, 0.0)
    rear_docking_pose = create_pose(rear_nav, 3.5, -2.0, 0.0)

    # 두 로봇에게 동시에 명령 하달 (비동기)
    front_nav.goToPose(front_docking_pose)
    rear_nav.goToPose(rear_docking_pose)

    # 두 로봇이 모두 목표에 도달할 때까지 대기
    while not front_nav.isTaskComplete() or not rear_nav.isTaskComplete():
        time.sleep(0.5)

    bridge_commander.get_logger().info("✅ 정렬 완료! 물리적 도킹을 시도합니다.")
    
    # 1번 카트 결합 시그널 발송 (브리지 코드 매핑 기준: 4=프론트-카트1, 1=리어-카트1)
    attach_msg = Int32()
    attach_msg.data = 4
    attach_pub.publish(attach_msg)
    time.sleep(1.0)
    
    attach_msg.data = 1
    attach_pub.publish(attach_msg)
    time.sleep(2.0) # 찰칵! 도킹되는 시간 대기

    # =====================================================================
    # [Step 5] 기차 모드로 출발점 복귀 (Long Ackermann 모드)
    # =====================================================================
    bridge_commander.get_logger().info("▶️ [Step 5] 로봇-카트-로봇 기차 결합 완료. 출발점으로 복귀합니다.")
    
    # 🚨 핵심: 파이썬 브리지가 '아커만 기구학(wheelbase 연장)'을 쓰도록 다시 is_attached = True로 만듦
    detach_msg.data = False 
    detach_pub.publish(detach_msg)
    time.sleep(1.0)

    # 마스터인 리어봇에게 출발점(Home) 좌표 명령
    home_pose = create_pose(rear_nav, 0.0, 0.0, 3.14)
    rear_nav.goToPose(home_pose)

    while not rear_nav.isTaskComplete():
        time.sleep(1.0)

    if rear_nav.getResult() == TaskResult.SUCCEEDED:
        bridge_commander.get_logger().info("🎉🎉🎉 모든 시나리오 임무 완수! 복귀 성공! 🎉🎉🎉")
    else:
        bridge_commander.get_logger().error("❌ 복귀 중 문제가 발생했습니다.")

    # 종료
    bridge_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()