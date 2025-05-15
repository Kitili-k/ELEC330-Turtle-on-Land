#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray


class AutonomousExplorer(Node):
    """Reactive explorer + PID‑based tracker + weighted obstacle avoidance."""

   
    def __init__(self) -> None:
        super().__init__("autonomous_explorer")

        
        self.declare_parameter("linear_speed", 0.25)       # m/s cruise
        self.declare_parameter("angular_speed", 1.5)       # rad/s saturate
        self.declare_parameter("angular_kp", 0.6)          # P gain
        self.declare_parameter("angular_ki", 0.05)         # I gain
        self.declare_parameter("angular_kd", 0.15)         # D gain
        self.declare_parameter("angular_i_max", 10.0)      # integral clamp
        self.declare_parameter("danger_dist", 1.7)         # m – slow‑down / weighting start
        self.declare_parameter("stop_dist", 0.3)           # m – full avoid
        self.declare_parameter("fov_deg", 100.0)            # laser sector FOV
        self.declare_parameter("img_width", 800)           # px – frame width
        self.declare_parameter("cam_freq", 5)              # Hz – camera rate

       
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan2", self._scan_cb, 10)
        self.create_subscription(Detection2DArray, "/camera", self._det_cb, 10)


        self.linear_speed: float = self.get_parameter("linear_speed").value
        self.angular_speed: float = self.get_parameter("angular_speed").value
        self.angular_kp: float = self.get_parameter("angular_kp").value
        self.angular_ki: float = self.get_parameter("angular_ki").value
        self.angular_kd: float = self.get_parameter("angular_kd").value
        self.angular_i_max: float = self.get_parameter("angular_i_max").value
        self.danger_dist: float = self.get_parameter("danger_dist").value
        self.stop_dist: float = self.get_parameter("stop_dist").value
        self.fov_deg: float = self.get_parameter("fov_deg").value
        self.img_width: int = self.get_parameter("img_width").value
        self.cam_freq: float = self.get_parameter("cam_freq").value
        self.dt: float = 1.0 / max(self.cam_freq, 1e-3)


        self._last_scan: Optional[LaserScan] = None
        self._err_int: float = 0.0
        self._prev_err: float = 0.0

        self.get_logger().info("AutonomousExplorer ready – waiting for camera…")


    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_scan = msg


    def _det_cb(self, msg: Detection2DArray) -> None:
        """Blend navigation & avoidance based on obstacle proximity."""

        if self._last_scan is None:
            self._cmd_pub.publish(Twist())
            return


        if msg.detections:
            twist_nav = self._track(msg.detections)
        else:
            twist_nav = self._explore()


        twist_avoid = self._avoid()
        w = self._avoid_weight()  # 0‑1


        twist = Twist()
        twist.linear.x  = (1.0 - w) * twist_nav.linear.x  + w * twist_avoid.linear.x
        twist.angular.z = (1.0 - w) * twist_nav.angular.z + w * twist_avoid.angular.z



        self._cmd_pub.publish(twist)

    def _scan_sector(self, half_deg: float) -> np.ndarray:
        """Return distance array within ±half_deg around the front."""
        scan = self._last_scan
        if scan is None:
            return np.array([])
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        mask = np.abs(np.rad2deg(angles)) <= half_deg
        sector = np.asarray(scan.ranges)[mask]

        max_dist = scan.range_max if math.isfinite(scan.range_max) else 10.0
        sector = np.where(np.isinf(sector), max_dist, sector)
        return sector


    def _nearest_obstacle(self) -> float:
        sector = self._scan_sector(self.fov_deg)
        if sector.size == 0:
            return math.inf
        nearest = float(np.nanmin(np.where(np.isfinite(sector), sector, np.inf)))
        return nearest


    def _avoid_weight(self) -> float:
        """Compute blending weight **w ∈ [0,1]** based on nearest obstacle."""
        d = self._nearest_obstacle()
        if not math.isfinite(d) or d >= self.danger_dist:
            return 0.0
        if d <= self.stop_dist:
            return 1.0
        # Linear interpolation between stop_dist → danger_dist
        return (self.danger_dist - d) / (self.danger_dist - self.stop_dist)


    def _limit_speed(self, twist: Twist) -> None:
        """Clamp forward speed to avoid collisions (safety fallback)."""
        if twist.linear.x <= 0.0:
            return
        sector = self._scan_sector(self.fov_deg)
        if sector.size == 0:
            return
        nearest = float(np.nanmin(np.where(np.isfinite(sector), sector, np.inf)))
        if not math.isfinite(nearest):
            return
        if nearest <= self.stop_dist:
            twist.linear.x = 0.1
        elif nearest <= self.danger_dist:
            scale = (nearest - self.stop_dist) / (self.danger_dist - self.stop_dist)
            twist.linear.x *= max(0.1, min(scale, 1.0))

    def _explore(self) -> Twist:
        """Simple forward‑biased wandering (low‑level obstacle reactive)."""
        twist = Twist()
        if self._last_scan is None:
            return twist

        sector = self._scan_sector(15.0)        # ±30° front window
        front_min = float(np.nanmin(np.where(np.isfinite(sector), sector, np.inf)))

        if front_min < self.stop_dist:
            # Immediate pivot to escape
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            # Gentle steering to more open side
            left = np.nanmean(sector[: sector.size // 2])
            right = np.nanmean(sector[sector.size // 2 :])
            twist.angular.z = self.angular_speed * (
                (right - left) / max(left, right, 1e-6)
            )
        return twist


    def _avoid(self) -> Twist:
        """Obstacle‑repulsive behaviour used for weighted blending."""
        twist = Twist()
        if self._last_scan is None:
            return twist

        nearest = self._nearest_obstacle()
        if not math.isfinite(nearest):
            return twist

        # Compute angular escape direction – steer away from denser side
        sector = self._scan_sector(self.fov_deg)
        if sector.size == 0:
            return twist

        left = np.nanmean(sector[: sector.size // 2])
        right = np.nanmean(sector[sector.size // 2 :])
        twist.angular.z = self.angular_speed * (
            (right - left) / max(left, right, 1e-6)
        )

        # Reduce forward speed proportionally to proximity
        if nearest <= self.stop_dist:
            twist.linear.x = 0.05
        else:
            scale = max(0.05, min((nearest - self.stop_dist) / (self.danger_dist - self.stop_dist), 1.0))
            twist.linear.x = self.linear_speed * scale * 0.5  # slower than cruise
        return twist


    def _track(self, detections) -> Twist:
        """Follow the most confident detection using PID control on heading."""
        det = detections[0]
        bbox = det.bbox
        cx_px: float = bbox.center.position.x  # bbox centre (px)
        err = (cx_px - self.img_width / 2) / (self.img_width / 2)  # ∈[-1,1]

        # PID terms
        self._err_int += err * self.dt
        self._err_int = max(-self.angular_i_max, min(self._err_int, self.angular_i_max))
        derr = (err - self._prev_err) * self.cam_freq
        self._prev_err = err

        omega = -(
            self.angular_kp * err
            + self.angular_ki * self._err_int
            + self.angular_kd * derr
        )
        omega = max(-self.angular_speed, min(omega, self.angular_speed))

        # Heuristic linear speed (slow down on large error or near target)
        est_dist = bbox.size_x / (0.6 * self.img_width)
        lin = self.linear_speed * (1.0 - min(max(est_dist, 0.0), 1.0))
        if abs(err) >= 0.4:
            lin *= 1.0 - min(abs(err), 1.0)

        twist = Twist()
        twist.linear.x = max(0.0, lin)
        twist.angular.z = omega
        return twist



def main() -> None:
    rclpy.init()
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
