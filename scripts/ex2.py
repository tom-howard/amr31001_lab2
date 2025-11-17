#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr31001_lab2_modules.tb3_tools import Motion, Pose
from math import sqrt, pow

class Square(Node):

    def __init__(self):
        super().__init__("move_square")

        self.motion = Motion(self)
        self.pose = Pose(self)

        self.turn = False 
        
        self.create_timer(
            timer_period_sec=0.1, # 10 Hz
            callback=self.move_square,
        )

        self.xpos_ref = 0.0; self.ypos_ref = 0.0; self.yaw_ref = 0.0
        self.yaw = 0.0 # a variable to keep track of how far the robot has turned
        self.displacement = 0.0 # a variable to keep track of how far the robot has moved
             
        self.shutdown = False

    def on_shutdown(self):
        print("Stopping the robot...")
        self.motion.stop()
        self.shutdown = True

    def move_square(self):
        if self.turn:
            # turn by 90 degrees...
            # keep track of how much yaw has been accrued during the current turn
            self.yaw = self.yaw + abs(self.pose.yaw - self.yaw_ref)
            self.yaw_ref = self.pose.yaw
            if self.yaw > 90:
                # That's enough, stop turning!
                self.motion.stop()
                self.turn = False
                self.yaw = 0.0
                self.xpos_ref = self.pose.posx
                self.ypos_ref = self.pose.posy
            else:
                # Not there yet, keep going:
                self.motion.move_at_velocity(
                    linear=0.0, angular=0.3
                )
        else:
            # move forwards by 1m...
            # keep track of how much displacement has been accrued so far
            # (Note: Euclidean Distance)
            self.displacement = self.displacement + sqrt(pow(self.pose.posx-self.xpos_ref, 2) + pow(self.pose.posy-self.ypos_ref, 2))
            self.xpos_ref = self.pose.posx
            self.ypos_ref = self.pose.posy
            if self.displacement >= 1:
                # That's enough, stop moving!
                self.motion.stop()
                self.turn = True
                self.displacement = 0.0
                self.yaw_ref = self.pose.yaw
            else:
                # Not there yet, keep going:
                self.motion.move_at_velocity(
                    linear=0.1, angular=0.0
                )

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Square()
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