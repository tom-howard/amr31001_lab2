#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr31001_lab2_modules.tb3_tools import Motion, Camera

class LineFollower(Node):
    
    def __init__(self):
        super().__init__("line_follower")

        self.motion = Motion(self)
        self.camera = Camera(self)

        self.camera.colour_filter()

        self.create_timer(
            timer_period_sec=0.2, 
            callback=self.follow_line,
        )
    
        self.shutdown = False
        
    def shutdown_ops(self):
        print("Stopping the robot...")
        self.camera.close()
        self.motion.stop()
        self.shutdown = True

    def follow_line(self):
        
        if self.camera.waiting_for_images:
            return

        kp = 0.01
        reference_input = self.camera.image_width / 2
        error = 0.0 # TODO

        ang_vel = kp * error
        
        self.motion.move_at_velocity()

        self.get_logger().info(
            f"\nLine offset = {error:.1f} pixels"
            f"\nAngular velocity = {ang_vel:.3f} rad/s.",
            throttle_duration_sec=1.0
        )
        
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"{node.get_name()} received a shutdown request (Ctrl+C)"
        )
    finally:
        node.shutdown_ops()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
