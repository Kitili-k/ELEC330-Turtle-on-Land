#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import gz.msgs
from geometry_msgs import Pose
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PoseToOdomNode(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
       
        self.pose_sub = self.create_subscription(
            Pose_V,
            '/world/fws_robot_world/pose/info',  
            self.pose_callback,
            qos_profile
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/world/fws_robot_world/pose/info',
            self.pise_callback,
            10
        )
        
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        
        self.last_pose = None
        self.last_time = None
        
        self.get_logger().info('Pose to Odometry converter node has been initialized')

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        
       
        robot_pose = None
        for pose in msg.pose:
            if pose.name == 'turtle':  
                robot_pose = pose
                break
                
        if robot_pose is None:
            return
            
      
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        
        odom.pose.pose.position = robot_pose.position
        odom.pose.pose.orientation = robot_pose.orientation
        
        
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                
                odom.twist.twist.linear.x = (robot_pose.position.x - self.last_pose.position.x) / dt
                odom.twist.twist.linear.y = (robot_pose.position.y - self.last_pose.position.y) / dt
                odom.twist.twist.linear.z = (robot_pose.position.z - self.last_pose.position.z) / dt
                
                
        
       
        self.odom_pub.publish(odom)
        
    
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = robot_pose.position.x
        t.transform.translation.y = robot_pose.position.y
        t.transform.translation.z = robot_pose.position.z
        t.transform.rotation = robot_pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        
        self.last_pose = robot_pose
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()