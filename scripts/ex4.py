#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import SaveMap

class MapSaver(Node):

    def __init__(self):
        super().__init__("map_saver")

        self.client = self.create_client(
            srv_type=SaveMap, 
            srv_name='map_saver/save_map'
        )

        self.create_timer(
            timer_period_sec=5, 
            callback=self.save_map,
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            continue

        self.get_logger().info(
            "Map service ready."
        )

    def save_map(self):
        pkg_share = get_package_share_directory('amr31001_lab2')
        # todo: get src dir of pkg and cal map service to save map

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MapSaver()
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