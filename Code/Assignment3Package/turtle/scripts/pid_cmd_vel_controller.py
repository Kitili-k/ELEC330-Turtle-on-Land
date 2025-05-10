#!/usr/bin/env python3


import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection2DArray


class PID:
    """Simple discrete PID controller."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = abs(i_max)
        self._prev_err = 0.0
        self._integral = 0.0
        self._prev_time: Optional[Time] = None

    def reset(self):
        self._prev_err = 0.0
        self._integral = 0.0
        self._prev_time = None

    def update(self, err: float, now: Time) -> float:
        if self._prev_time is None:
            self._prev_time = now
            self._prev_err = err
            return self.kp * err

        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return 0.0

        self._integral += err * dt
        self._integral = max(-self.i_max, min(self.i_max, self._integral))

        d_err = (err - self._prev_err) / dt

        self._prev_err = err
        self._prev_time = now

        return (
            self.kp * err +
            self.ki * self._integral +
            self.kd * d_err
        )


class CmdVelPIDController(Node):
    def __init__(self):
        super().__init__('cmd_vel_pid_controller')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_width', 800),
                ('image_height', 600),
                ('linear_pid.kp', 1.2),
                ('linear_pid.ki', 0.0),
                ('linear_pid.kd', 0.1),
                ('angular_pid.kp', 2.5),
                ('angular_pid.ki', 0.0),
                ('angular_pid.kd', 0.15),
                ('max_linear_speed', 0.3),
                ('max_angular_speed', 2.0),
                ('control_rate', 30.0)
            ]
        )

        self.linear_pid = PID(
            kp=self.get_parameter('linear_pid.kp').value,
            ki=self.get_parameter('linear_pid.ki').value,
            kd=self.get_parameter('linear_pid.kd').value,
            i_max=0.5,
        )
        self.angular_pid = PID(
            kp=self.get_parameter('angular_pid.kp').value,
            ki=self.get_parameter('angular_pid.ki').value,
            kd=self.get_parameter('angular_pid.kd').value,
            i_max=1.0,
        )

        self._have_target = False
        self._desired_lin_vel = 0.0
        self._desired_ang_vel = 0.0
        self._actual_lin_vel = 0.0
        self._actual_ang_vel = 0.0
        self._prev_imu_time: Optional[Time] = None

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Detection2DArray, '/camera', self._bbox_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 50)

        control_dt = 1.0 / self.get_parameter('control_rate').value
        self.create_timer(control_dt, self._control_step)

        self.get_logger().info('PID cmd_vel controller initialised (first‑bbox mode)')



    def _bbox_cb(self, msg: Detection2DArray):
        # If there is at least one detection, use the first one
        if not msg.detections:
            self._have_target = False
            return

        det = msg.detections[0]
        self._have_target = True

        img_w = float(self.get_parameter('image_width').value)
        img_h = float(self.get_parameter('image_height').value)

        center_x = det.bbox.center.position.x
        width_x = det.bbox.size_x
        # self.get_logger().info(f'target center point: ({center_x})')

        x_offset = center_x - img_w / 2.0
        norm_x = x_offset / (img_w / 2.0)

        dist_factor = width_x / (0.6 * img_w)

        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value

        self._desired_ang_vel = -max_ang * norm_x
        self._desired_lin_vel = max_lin * (1.0 - dist_factor)
        if abs(norm_x) >= 0.5:
            self._desired_lin_vel = max_lin * (1 - abs(norm_x))

        self._last_bbox_time = self.get_clock().now()

    def _imu_cb(self, msg: Imu):
        now_msg = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        now_time = Time.from_msg(now_msg)

        self._actual_ang_vel = msg.angular_velocity.z

        if self._prev_imu_time is not None:
            dt = (now_time - self._prev_imu_time).nanoseconds * 1e-9
            accel_x = msg.linear_acceleration.x
            self._actual_lin_vel += accel_x * dt
            self._actual_lin_vel *= 0.99
        self._prev_imu_time = now_time
        self._last_imu_time = self.get_clock().now()



    def _control_step(self):
        now = self.get_clock().now()
        cmd = Twist()
        self.get_logger().info(f'desire:{self._desired_lin_vel}')
        bbox_stale = (now - self._last_bbox_time).nanoseconds *1e-9 > 0.2
        imu_stale = (now - self._last_imu_time).nanoseconds*1e-9> 0.05

        if bbox_stale:
            self._desired_lin_vel = 0
            self._desired_ang_vel = 0
            self.linear_pid.reset()
            self.angular_pid.reset()

        if imu_stale:
            self.linear_pid.ki = 0
            self.angular_pid.ki = 0

        if not self._have_target:
            self.linear_pid.reset()
            self.angular_pid.reset()
            self._desired_lin_vel = 0.0
            self._desired_ang_vel = 0.0

        lin_err = self._desired_lin_vel - self._actual_lin_vel
        lin_cmd = self.linear_pid.update(lin_err, now)

        ang_err = self._desired_ang_vel - self._actual_ang_vel
        ang_cmd = self.angular_pid.update(ang_err, now)

        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        lin_cmd = max(-max_lin, min(max_lin, lin_cmd))
        ang_cmd = max(-max_ang, min(max_ang, ang_cmd))

        cmd.linear.x = lin_cmd
        cmd.angular.z = ang_cmd
        self._pub_cmd.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
