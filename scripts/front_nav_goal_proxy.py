#!/usr/bin/env python3

import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
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
        self.declare_parameter("clear_cmd_vel_topic", "/front/cmd_vel")
        self.declare_parameter("clear_odom_topic", "/front/odom")
        self.declare_parameter("clear_control_period_sec", 0.05)

        self.server_timeout_sec = max(
            0.1,
            float(self.get_parameter("server_timeout_sec").value),
        )
        self.clear_control_period_sec = max(
            0.02,
            float(self.get_parameter("clear_control_period_sec").value),
        )
        self.active_goal_id = None
        self.active_goal_handle = None
        self.active_goal_seq = 0
        self.active_clear_goal = None
        self.clear_timer = None
        self.last_odom = None

        goal_topic = str(self.get_parameter("goal_topic").value)
        cancel_topic = str(self.get_parameter("cancel_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        action_name = str(self.get_parameter("action_name").value)
        clear_cmd_vel_topic = str(self.get_parameter("clear_cmd_vel_topic").value)
        clear_odom_topic = str(self.get_parameter("clear_odom_topic").value)

        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.clear_cmd_pub = self.create_publisher(Twist, clear_cmd_vel_topic, 10)
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
        self.odom_sub = self.create_subscription(
            Odometry,
            clear_odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            "front nav goal proxy ready: "
            f"{goal_topic} -> {action_name}, "
            f"clear_cmd={clear_cmd_vel_topic}, result={result_topic}"
        )

    def goal_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            goal_id = int(payload["id"])
        except Exception as ex:
            self.get_logger().warn(f"invalid front proxy goal: {ex}")
            return

        if str(payload.get("command", "")).strip().lower() == "clear_forward":
            self.start_clear_forward(goal_id, payload)
            return

        try:
            pose = self.payload_to_pose(payload)
        except Exception as ex:
            self.get_logger().warn(f"invalid front proxy goal pose: {ex}")
            return

        self.cancel_active_goal()

        if not self.nav_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            self.get_logger().warn(
                f"front Nav2 action server not ready for goal id={goal_id}"
            )
            self.publish_result(goal_id, False, "front Nav2 action server not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.active_goal_id = goal_id
        self.active_goal_seq += 1
        goal_seq = self.active_goal_seq

        try:
            self.get_logger().info(
                "front goal received: "
                f"id={goal_id}, frame={pose.header.frame_id}, "
                f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}"
            )
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
        self.get_logger().info(f"front goal cancel requested: id={goal_id}")
        self.publish_result(goal_id, False, "front goal canceled")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def start_clear_forward(self, goal_id: int, payload: dict):
        self.cancel_active_goal()

        if self.last_odom is None:
            self.get_logger().warn(
                f"front clear rejected: odom not available for id={goal_id}"
            )
            self.publish_result(goal_id, False, "front odom not available")
            return

        distance = max(0.0, float(payload.get("distance", 0.5)))
        speed = max(0.01, abs(float(payload.get("speed", 0.12))))
        timeout_sec = max(0.5, float(payload.get("timeout_sec", 8.0)))
        start_x, start_y = self.odom_xy()

        self.active_goal_id = goal_id
        self.active_goal_seq += 1
        self.active_clear_goal = {
            "id": goal_id,
            "seq": self.active_goal_seq,
            "start_x": start_x,
            "start_y": start_y,
            "distance": distance,
            "speed": speed,
            "deadline": self.get_clock().now().nanoseconds * 1e-9 + timeout_sec,
        }

        self.get_logger().info(
            "front clear started: "
            f"id={goal_id}, distance={distance:.2f}m, speed={speed:.2f}m/s"
        )
        self.clear_timer = self.create_timer(
            self.clear_control_period_sec,
            self.update_clear_forward,
        )

    def update_clear_forward(self):
        clear_goal = self.active_clear_goal
        if clear_goal is None:
            self.stop_clear_timer()
            return

        if self.last_odom is None:
            self.finish_clear_forward(False, "front odom lost")
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        current_x, current_y = self.odom_xy()
        traveled = math.hypot(
            current_x - clear_goal["start_x"],
            current_y - clear_goal["start_y"],
        )

        if traveled >= clear_goal["distance"]:
            self.finish_clear_forward(True, f"traveled={traveled:.3f}m")
            return

        if now_sec >= clear_goal["deadline"]:
            self.finish_clear_forward(False, f"timeout traveled={traveled:.3f}m")
            return

        cmd = Twist()
        cmd.linear.x = clear_goal["speed"]
        self.clear_cmd_pub.publish(cmd)

    def finish_clear_forward(self, success: bool, message: str):
        clear_goal = self.active_clear_goal
        if clear_goal is None:
            return

        goal_id = int(clear_goal["id"])
        self.publish_stop()
        self.stop_clear_timer()
        self.active_clear_goal = None
        self.clear_active_goal()

        if success:
            self.get_logger().info(f"front clear succeeded: id={goal_id}, {message}")
        else:
            self.get_logger().warn(f"front clear failed: id={goal_id}, {message}")
        self.publish_result(goal_id, success, message)

    def odom_xy(self):
        position = self.last_odom.pose.pose.position
        return (float(position.x), float(position.y))

    def publish_stop(self):
        stop = Twist()
        for _ in range(3):
            self.clear_cmd_pub.publish(stop)

    def stop_clear_timer(self):
        if self.clear_timer is not None:
            self.clear_timer.cancel()
            self.clear_timer = None

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
            self.get_logger().warn(f"front goal rejected: id={goal_id}")
            self.publish_result(goal_id, False, "front goal rejected")
            self.clear_active_goal()
            return

        self.get_logger().info(f"front goal accepted: id={goal_id}")
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
            self.get_logger().info(f"front goal succeeded: id={goal_id}")
            self.publish_result(goal_id, True, "succeeded")
        else:
            status = "None" if result is None else str(result.status)
            self.get_logger().warn(f"front goal finished without success: id={goal_id}, {status}")
            self.publish_result(goal_id, False, f"status={status}")

        self.clear_active_goal()

    def cancel_active_goal(self):
        self.active_goal_seq += 1
        if self.active_clear_goal is not None:
            self.publish_stop()
            self.stop_clear_timer()
            self.active_clear_goal = None
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()


# source /home/inyup/colcon_ws/install/setup.bash
# ROS_DOMAIN_ID=48 python3 /home/inyup/colcon_ws/src/cap_sim_2026/scripts/front_nav_goal_proxy.py
