from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from math import atan2, asin, degrees
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

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
                f"LINEAR velocity limited to {linear} m/s ({lin_org} m/s was requested).",
                throttle_duration_sec=2.0
            )
        ang_lim = 1.0 # rad/s
        if abs(angular) > ang_lim:
            ang_org = angular
            angular = (angular / abs(angular)) * ang_lim
            self.node.get_logger().warning(
                f"ANGULAR velocity limited to {angular} rad/s ({ang_org} rad/s was requested).",
                throttle_duration_sec=2.0
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

class Lidar():
    def __init__(self, node: Node):
        self.distance = self.scanSubsets(node)
        self.subscriber = node.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.laserscan_cb,
            qos_profile=10
        )  

    class scanSubsets():
        def __init__(self, node: Node):
            self.node = node
            self.front = 0.0
            self.r1 = 0.0; self.r2 = 0.0; self.r3 = 0.0; self.r4 = 0.0
            self.l1 = 0.0; self.l2 = 0.0; self.l3 = 0.0; self.l4 = 0.0
        
        def show(self):
            lidar_segs = f"              l1     front     r1          \n" \
                         f"       l2     {self.l1:<5.3f}  {self.front:^5.3f}  {self.r1:>5.3f}     r2       \n" \
                         f"l3     {self.l2:<5.3f}                       {self.r2:>5.3f}   r3\n" \
                         f"{self.l3:<5.3f}                                   {self.r3:>5.3f}\n" \
                         f"{self.l4:<5.3f} <-- l4                     r4 --> {self.r4:>5.3f}"
            
            return lidar_segs
                        
    def laserscan_cb(self, scan_data: LaserScan):

        def subset_average(start_index, stop_index):
            range = np.array(scan_data.ranges[start_index: stop_index+1])
            valid_data = range[range>0.1]
            return valid_data.mean() if np.shape(valid_data)[0] > 0 else np.nan

        # front:
        left_arc = scan_data.ranges[0:20+1]
        right_arc = scan_data.ranges[-20:]
        full_arc = np.array(left_arc[::-1] + right_arc[::-1])
        valid = full_arc[full_arc>0.1]
        self.distance.front = valid.mean() if np.shape(valid)[0] > 0 else np.nan
        
        # right subsets:
        self.distance.r1 = subset_average(320, 340)
        self.distance.r2 = subset_average(300, 320)
        self.distance.r3 = subset_average(275, 290)
        self.distance.r4 = subset_average(250, 265)
        
        # left subsets:
        self.distance.l1 = subset_average(20, 40)
        self.distance.l2 = subset_average(40, 60)
        self.distance.l3 = subset_average(70, 85)
        self.distance.l4 = subset_average(95, 110)
        
        self.wait_for_readings = False

class Camera():
    def __init__(self, node: Node):
        self.node = node
        
        self.subscriber = self.node.create_subscription(
            msg_type=Image,
            topic='/camera/color/image_raw',
            callback=self.cam_cb,
            qos_profile=10
        )
        self.colour_detected = False
        self.m00_threshold = 20000
        self.colour_filter()
        self.waiting_for_images = True  

    def colour_filter(self, 
            hue = [0, 225], 
            saturation = [0, 255], 
            value = [0, 255]):
        self.lower = (min(hue), min(saturation), min(value))
        self.upper = (max(hue), max(saturation), max(value)) 
    
    def cam_cb(self, img_data: Image):
        cvbridge_interface = CvBridge()
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"{e}")

        height, width, _ = cv_img.shape
        self.image_width = width
        
        if height > 480: 
            crop_height = int(height / 5)
            crop_z0 = height - 200 - crop_height
            crop_z1 = crop_z0 + crop_height
            cropped_img = cv_img[
                crop_z0:crop_z1, 0:width
            ]
        else:
            cropped_img = cv_img
        
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        line_mask = cv2.inRange(
            hsv_img, self.lower, self.upper
        )
        
        m = cv2.moments(line_mask)
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5)

        self.colour_detected = True if m['m00'] > self.m00_threshold else False
        if not self.colour_detected:
            self.node.get_logger().warning(
                "No colour blob detected within the specified HSV range.",
                throttle_duration_sec=2.0
            )
            self.line_position_pixels = 0.0
        else:
            self.line_position_pixels = cy

        res = cv2.bitwise_and(cropped_img, cropped_img, mask = line_mask)

        if self.colour_detected:
            cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
        
        cv2.imshow("filtered image", res)
        cv2.waitKey(1)
        
        self.waiting_for_images = False
    
    def close(self):
        cv2.destroyAllWindows()