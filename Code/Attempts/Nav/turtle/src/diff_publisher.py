import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Subscribe to the /cmd_vel topic (from teleop_twist_keyboard)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish to the /diff_drive/cmd_vel topic as TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive/cmd_vel', 10)

        self.get_logger().info("cmd_vel_bridge node started. Subscribing to /cmd_vel and publishing to /diff_drive/cmd_vel")

    def cmd_vel_callback(self, msg: Twist):
        # Convert Twist to TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "odom"  # or "base_link" depending on your setup
        twist_stamped.twist = msg

        self.publisher_.publish(twist_stamped)

        self.get_logger().info(
            f"Bridged: linear x = {msg.linear.x:.2f}, angular z = {msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
