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

    def follow_wall(self):
        lin_vel = 0.1

        wall_slope = self.lidar.distance.l3 - self.lidar.distance.l4
        self.lidar.distance.show()

        if (self.lidar.distance.front < 0.3) or (self.lidar.distance.l1 < 0.4):
            lin_vel = 0.0
            ang_vel = -0.3
            movement = "turning to avoid collision up ahead..."
        elif (self.lidar.distance.l3 > 0.6):
            movement = "lost sight of the wall, turning left..."
            lin_vel = 0.0
            ang_vel = 0.3
        elif abs(wall_slope) < 0.001:
            movement = "go straight"
            ang_vel = 0.0
        elif wall_slope < 0:
            movement = "turn right"
            ang_vel = -0.2 if self.lidar.distance.l3 > 0.2 else -0.4
        else:
            movement = "turn left"
            ang_vel = 0.2 if self.lidar.distance.l4 < 0.2 else 0.4
        
        self.get_logger().info("\n"
            f"{wall_slope=:.3f}"
            f"{movement=}\n"
        )

        self.motion.move_at_velocity(linear=lin_vel, angular=ang_vel)

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