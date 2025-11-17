from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Quaternion
from nav_msgs.msg import Odometry
from math import atan2, asin, degrees
import numpy as np

def quaternion_to_euler(orientation: Quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    adapted from:
    https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    
    return roll, pitch, yaw # in radians

class Motion():
    def __init__(self, node: Node):
        self.node = node
        self.publisher = self.node.create_publisher(
            TwistStamped, 
            'cmd_vel',
            10
        )
        
        self.vel_cmd = TwistStamped()
    
    def move_at_velocity(self, linear = 0.0, angular = 0.0):
        if abs(linear) > 0.26:
            lin_org = linear
            linear = (linear / abs(linear)) * 0.26
            self.node.get_logger().warning(
                f"LINEAR velocity limited to {linear} m/s ({lin_org} m/s was requested)."
            )

        if abs(angular) > 1.82:
            ang_org = angular
            angular = (angular / abs(angular)) * 1.82
            self.node.get_logger().warning(
                f"ANGULAR velocity limited to {angular} rad/s ({ang_org} rad/s was requested)."
            )
        
        self.vel_cmd.twist.linear.x = linear
        self.vel_cmd.twist.angular.z = angular
        self.publish()
    
    def stop(self):
        self.move_at_velocity()
        self.publish()
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
class Pose():
    def __init__(self, node: Node):
        self.node = node
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.yaw_direction = 0.0
        
        self.subscriber = self.node.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.odom_cb,
            qos_profile=10
        )

        self.timestamp = self.node.get_clock().now().nanoseconds
    
    def odom_cb(self, odom_data: Odometry):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = quaternion_to_euler(orientation)
        
        yaw = self.round(degrees(yaw), 4)
        self.yaw_direction = np.sign(yaw)
        self.yaw = abs(yaw)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

        self.wait_for_odom = False
    
    def show(self):
        now = self.node.get_clock().now().nanoseconds
        if now - self.timestamp > 1e9:
            self.timestamp = self.node.get_clock().now().nanoseconds
            self.node.get_logger().info(
                f"posx = {self.posx:.3f} (m), "
                f"posy = {self.posy:.3f} (m), "
                f"yaw = {self.yaw:.1f} (degrees)"
            )
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)
        