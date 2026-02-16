#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr31001_lab2_modules.tb3_tools import Motion, Lidar

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")

        self.motion = Motion(self)
        self.lidar = Lidar(self)

        self.create_timer(
            timer_period_sec=0.1, 
            callback=self.follow_wall,
        )
    
    def on_shutdown(self):
        print("Stopping the robot...")
        self.motion.stop()
        self.shutdown = True

    def follow_wall(self):
        lin_vel = 0.0

        wall_slope = self.lidar.distance.l3 - self.lidar.distance.l4

        if abs(wall_slope) < 0.001:
            action = "go straight"
            ang_vel = 0.0
        elif wall_slope < 0:
            action = "turn left, or right??"
            ang_vel = 0.0
        else:
            action = "turn left, or right??"
            ang_vel = 0.0
        
        self.get_logger().info("\n"
            f"{self.lidar.distance.show()}\n"
            f"{wall_slope=:.3f}\n"
            f"{action=}\n",
            throttle_duration_sec=1.0
        )

        self.motion.move_at_velocity(
            linear=lin_vel, angular=ang_vel
        )

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()