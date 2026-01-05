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

        self.camera.colour_filter(
            # hue=[95, 105],
            # saturation=[150, 255],
        )

        self.create_timer(
            timer_period_sec=0.1, 
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
            self.get_logger().info(
                "Waiting for images...",
                throttle_duration_sec=1.0
            )
            return

        kp = -0.001
        error = self.camera.line_error_pixels 

        ang_vel = kp * error
        lin_vel = 0.1 if self.camera.colour_detected else 0.0
        
        self.motion.move_at_velocity(
            linear = lin_vel,
            angular = ang_vel
        )

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
