#!/usr/bin/env python3
from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from nav2_msgs.action import FollowWaypoints
import tf2_ros



def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (rad) into a ROS `Quaternion`."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw (ignoring roll & pitch) from a `Quaternion`."""
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def wrap_to_pi(angle: float) -> float:
    """Wrap `angle` to the interval [‑π, π)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi




class SimpleWaypointNav(Node):
    """Converts camera detections into a single waypoint for Nav2."""

    def __init__(self) -> None:
        super().__init__('simple_target_to_waypoint')

    
        self.declare_parameter('image_width', 800)
        self.declare_parameter('hfov_deg', 60.0)
        self.declare_parameter('goal_offset', 0.4)  # stop this far before the object
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.image_width: int = self.get_parameter('image_width').value
        self.hfov: float = math.radians(self.get_parameter('hfov_deg').value)
        self.goal_offset: float = self.get_parameter('goal_offset').value
        self.global_frame: str = self.get_parameter('global_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value


        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_scan: LaserScan | None = None
        self.create_subscription(LaserScan, '/scan2', self._scan_cb, 10)
        self.latest_goal_base: tuple[float, float] | None = None
        self.create_subscription(Detection2DArray, '/camera', self._camera_cb, 10)


        self.wp_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.sent_goal: PoseStamped | None = None


        self.create_timer(1.0, self._timer_cb)  # 1 Hz
        self.get_logger().info('SimpleWaypointNav initialised (publishing at 1 Hz).')


    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _camera_cb(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return

        det = msg.detections[0]
        cx = det.bbox.center.position.x
        x_offset_px = cx - self.image_width / 2.0
        bearing = (x_offset_px / (self.image_width / 2.0)) * (self.hfov / 2.0)
        bearing = wrap_to_pi(bearing)

        rng = self._range_at_bearing(bearing)
        if math.isinf(rng):
            rng = 10.0  # default when no return

        rng_adj = max(rng - self.goal_offset, 0.05)
        x_base = rng_adj * math.cos(bearing)
        y_base = rng_adj * math.sin(bearing)
        self.latest_goal_base = (x_base, y_base)

    def _timer_cb(self) -> None:
        if self.latest_goal_base is None:
            return
        if not self.wp_client.server_is_ready():
            self.get_logger().warn_throttle(5.0, 'follow_waypoints action server not ready.')
            return

        # Transform goal from base_link to map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn_throttle(5.0, 'TF transform unavailable.')
            return

        x_b, y_b = self.latest_goal_base
        yaw_base = quaternion_to_yaw(tf.transform.rotation)
        goal_x = tf.transform.translation.x + math.cos(yaw_base) * x_b - math.sin(yaw_base) * y_b
        goal_y = tf.transform.translation.y + math.sin(yaw_base) * x_b + math.cos(yaw_base) * y_b
        goal_yaw = wrap_to_pi(yaw_base + math.atan2(y_b, x_b))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation = yaw_to_quaternion(goal_yaw)

        if self._is_new_goal(pose, 0.15):
            self._send_waypoint(pose)


    def _range_at_bearing(self, bearing: float) -> float:
        """Return the LiDAR range at *bearing* (radians) or `inf`."""
        scan = self.latest_scan
        if scan is None:
            return math.inf
        index = int(round((bearing - scan.angle_min) / scan.angle_increment))
        if 0 <= index < len(scan.ranges):
            return scan.ranges[index]
        return math.inf

    def _is_new_goal(self, pose: PoseStamped, tol: float) -> bool:
        if self.sent_goal is None:
            return True
        dx = pose.pose.position.x - self.sent_goal.pose.position.x
        dy = pose.pose.position.y - self.sent_goal.pose.position.y
        return math.hypot(dx, dy) > tol

    def _send_waypoint(self, pose: PoseStamped) -> None:
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses.append(pose)
        self.get_logger().info('Sending waypoint: (%.2f, %.2f)', pose.pose.position.x, pose.pose.position.y)
        self.wp_client.send_goal_async(goal_msg)  # result is ignored for simplicity
        self.sent_goal = pose



def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
