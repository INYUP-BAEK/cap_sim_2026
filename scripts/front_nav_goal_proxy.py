#!/usr/bin/env python3

import json

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class FrontNavGoalProxy(Node):
    """Bridge topic goals to the local front NavigateToPose action.

    This node is intended to run in the front robot domain. The rear scenario
    runner can then use domain_bridge topic forwarding instead of trying to
    bridge a ROS action directly.
    """

    def __init__(self):
        super().__init__("front_nav_goal_proxy")

        self.declare_parameter("goal_topic", "/front/scenario_nav_goal")
        self.declare_parameter("cancel_topic", "/front/scenario_nav_cancel")
        self.declare_parameter("result_topic", "/front/scenario_nav_result")
        self.declare_parameter("action_name", "/front/navigate_to_pose")
        self.declare_parameter("server_timeout_sec", 2.0)

        self.server_timeout_sec = max(
            0.1,
            float(self.get_parameter("server_timeout_sec").value),
        )
        self.active_goal_id = None
        self.active_goal_handle = None
        self.active_goal_seq = 0

        goal_topic = str(self.get_parameter("goal_topic").value)
        cancel_topic = str(self.get_parameter("cancel_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.goal_sub = self.create_subscription(
            String,
            goal_topic,
            self.goal_callback,
            10,
        )
        self.cancel_sub = self.create_subscription(
            String,
            cancel_topic,
            self.cancel_callback,
            10,
        )

        self.get_logger().info(
            "front nav goal proxy ready: "
            f"{goal_topic} -> {action_name}, result={result_topic}"
        )

    def goal_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            goal_id = int(payload["id"])
            pose = self.payload_to_pose(payload)
        except Exception as ex:
            self.get_logger().warn(f"invalid front proxy goal: {ex}")
            return

        self.cancel_active_goal()

        if not self.nav_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            self.publish_result(goal_id, False, "front Nav2 action server not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.active_goal_id = goal_id
        self.active_goal_seq += 1
        goal_seq = self.active_goal_seq

        try:
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda fut, seq=goal_seq, gid=goal_id: self.goal_response(fut, seq, gid)
            )
        except Exception as ex:
            self.publish_result(goal_id, False, f"send_goal exception: {ex}")

    def cancel_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            goal_id = int(payload.get("id", -1))
        except Exception:
            return

        if self.active_goal_id is None or goal_id != self.active_goal_id:
            return

        self.cancel_active_goal()
        self.publish_result(goal_id, False, "front goal canceled")

    def goal_response(self, future, goal_seq: int, goal_id: int):
        if goal_seq != self.active_goal_seq or goal_id != self.active_goal_id:
            return

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.publish_result(goal_id, False, f"goal response exception: {ex}")
            self.clear_active_goal()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.publish_result(goal_id, False, "front goal rejected")
            self.clear_active_goal()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=goal_seq, gid=goal_id: self.goal_result(fut, seq, gid)
        )

    def goal_result(self, future, goal_seq: int, goal_id: int):
        if goal_seq != self.active_goal_seq or goal_id != self.active_goal_id:
            return

        try:
            result = future.result()
        except Exception as ex:
            self.publish_result(goal_id, False, f"goal result exception: {ex}")
            self.clear_active_goal()
            return

        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_result(goal_id, True, "succeeded")
        else:
            status = "None" if result is None else str(result.status)
            self.publish_result(goal_id, False, f"status={status}")

        self.clear_active_goal()

    def cancel_active_goal(self):
        self.active_goal_seq += 1
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.clear_active_goal()

    def clear_active_goal(self):
        self.active_goal_id = None
        self.active_goal_handle = None

    def publish_result(self, goal_id: int, success: bool, message: str):
        msg = String()
        msg.data = json.dumps(
            {
                "id": int(goal_id),
                "success": bool(success),
                "message": str(message),
            }
        )
        self.result_pub.publish(msg)

    def payload_to_pose(self, payload: dict):
        pose = PoseStamped()
        pose.header.frame_id = str(payload.get("frame_id") or "map")
        stamp = payload.get("stamp", {})
        pose.header.stamp.sec = int(stamp.get("sec", 0))
        pose.header.stamp.nanosec = int(stamp.get("nanosec", 0))

        position = payload.get("position", {})
        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))

        orientation = payload.get("orientation", {})
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = FrontNavGoalProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# source /home/inyup/colcon_ws/install/setup.bash
# ROS_DOMAIN_ID=48 python3 /home/inyup/colcon_ws/src/cap_sim_2026/scripts/front_nav_goal_proxy.py