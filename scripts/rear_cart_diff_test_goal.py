#!/usr/bin/env python3

import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Polygon, PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

import tf2_ros


class RearCartDiffTestGoal(Node):
    def __init__(self):
        super().__init__("rear_cart_diff_test_goal")

        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "")
        self.declare_parameter("base_frame_candidates", ["base_footprint", "base_link"])
        self.declare_parameter("back_distance_m", 1.0)
        self.declare_parameter("right_distance_m", 1.0)
        self.declare_parameter("send_delay_sec", 1.0)
        self.declare_parameter("server_timeout_sec", 5.0)
        self.declare_parameter("behavior_tree", "")
        self.declare_parameter("publish_rear_cart_footprint", True)
        self.declare_parameter("footprint_publish_period_sec", 0.5)
        self.declare_parameter("set_velocity_smoother", True)
        self.declare_parameter("velocity_smoother_node", "/velocity_smoother")

        self.action_name = str(self.get_parameter("action_name").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        self.base_frame_candidates = [
            str(frame).strip()
            for frame in self.get_parameter("base_frame_candidates").value
            if str(frame).strip()
        ]
        if self.base_frame:
            self.base_frame_candidates = [self.base_frame]

        self.back_distance_m = max(
            0.0,
            float(self.get_parameter("back_distance_m").value),
        )
        self.right_distance_m = float(self.get_parameter("right_distance_m").value)
        self.send_delay_sec = max(0.0, float(self.get_parameter("send_delay_sec").value))
        self.server_timeout_sec = max(
            0.1,
            float(self.get_parameter("server_timeout_sec").value),
        )
        self.publish_rear_cart_footprint = bool(
            self.get_parameter("publish_rear_cart_footprint").value
        )
        self.set_velocity_smoother = bool(
            self.get_parameter("set_velocity_smoother").value
        )
        self.velocity_smoother_node = str(
            self.get_parameter("velocity_smoother_node").value
        )

        behavior_tree = str(self.get_parameter("behavior_tree").value).strip()
        if behavior_tree:
            self.behavior_tree = behavior_tree
        else:
            self.behavior_tree = os.path.join(
                get_package_share_directory("cap_sim_2026"),
                "bt_xml",
                "rear_cart_diff_nav_tree.xml",
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        self.goal_handle = None

        self.global_footprint_pub = self.create_publisher(
            Polygon,
            "/global_costmap/footprint",
            10,
        )
        self.local_footprint_pub = self.create_publisher(
            Polygon,
            "/local_costmap/footprint",
            10,
        )

        if self.publish_rear_cart_footprint:
            period = max(
                0.1,
                float(self.get_parameter("footprint_publish_period_sec").value),
            )
            self.footprint_timer = self.create_timer(period, self.publish_footprint)
            self.publish_footprint()
        else:
            self.footprint_timer = None

        if self.set_velocity_smoother:
            self.set_rear_cart_velocity_limits()

        self.start_timer = self.create_timer(
            max(0.01, self.send_delay_sec),
            self.send_test_goal,
        )
        self.get_logger().info(
            "rear-cart diff test ready: goal is %.2fm back and %.2fm right, BT=%s"
            % (self.back_distance_m, self.right_distance_m, self.behavior_tree)
        )

    def send_test_goal(self):
        self.start_timer.cancel()
        self.destroy_timer(self.start_timer)

        pose = self.current_pose_in_map()
        if pose is None:
            self.get_logger().error(
                "failed to get current rear pose from TF frames: %s"
                % ", ".join(self.base_frame_candidates)
            )
            rclpy.shutdown()
            return

        x, y, yaw, frame = pose
        dx = -self.back_distance_m
        dy = -abs(self.right_distance_m)
        goal_x = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        goal_y = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        goal = self.create_pose_stamped(goal_x, goal_y, yaw)

        if not self.nav_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            self.get_logger().error(
                "NavigateToPose server is not ready: %s" % self.action_name
            )
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        goal_msg.behavior_tree = self.behavior_tree

        self.get_logger().info(
            "sending rear-cart diff test goal from %s: "
            "current=(%.3f, %.3f, %.3f), goal=(%.3f, %.3f, %.3f)"
            % (frame, x, y, yaw, goal_x, goal_y, yaw)
        )
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("rear-cart diff test goal rejected.")
            rclpy.shutdown()
            return

        self.goal_handle = goal_handle
        self.get_logger().info("rear-cart diff test goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result()
        status = GoalStatus.STATUS_UNKNOWN if result is None else result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("rear-cart diff test goal succeeded.")
        else:
            self.get_logger().error(
                "rear-cart diff test goal failed: status=%s" % status
            )
        rclpy.shutdown()

    def current_pose_in_map(self):
        for frame in self.base_frame_candidates:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
            except tf2_ros.TransformException:
                continue

            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
            return (t.x, t.y, yaw, frame)
        return None

    def create_pose_stamped(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def publish_footprint(self):
        msg = self.create_polygon(front_x=1.60, rear_x=-0.30, width=0.25)
        self.global_footprint_pub.publish(msg)
        self.local_footprint_pub.publish(msg)

    def set_rear_cart_velocity_limits(self):
        params = {
            "max_velocity": [0.0, 0.0, 0.30],
            "min_velocity": [-0.12, 0.0, -0.30],
            "max_accel": [0.20, 0.0, 1.0],
            "max_decel": [-0.20, 0.0, -1.0],
        }
        client = self.create_client(
            SetParameters,
            f"{self.velocity_smoother_node}/set_parameters",
        )
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "%s set_parameters service not ready." % self.velocity_smoother_node
            )
            return

        request = SetParameters.Request()
        for name, value in params.items():
            request.parameters.append(
                Parameter(name, value=value).to_parameter_msg()
            )
        future = client.call_async(request)
        future.add_done_callback(self.velocity_params_done)

    def velocity_params_done(self, future):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().warn("velocity smoother parameter update failed: %s" % ex)
            return

        rejected = [result.reason for result in response.results if not result.successful]
        if rejected:
            self.get_logger().warn("velocity smoother rejected params: %s" % rejected)
        else:
            self.get_logger().info("velocity smoother set to rear-cart diff limits.")

    @staticmethod
    def create_polygon(front_x: float, rear_x: float, width: float):
        poly = Polygon()
        poly.points = [
            Point32(x=float(front_x), y=float(-width), z=0.0),
            Point32(x=float(front_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(width), z=0.0),
            Point32(x=float(rear_x), y=float(-width), z=0.0),
        ]
        return poly

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = RearCartDiffTestGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
