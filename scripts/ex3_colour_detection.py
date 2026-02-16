#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.task import Future
from ament_index_python.packages import get_package_share_directory

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 

import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
from pathlib import Path 

class ImageCapture(Node):

    def __init__(self): 
        super().__init__("object_detection")

        base_image_path = Path(
            get_package_share_directory("amr31001_lab2")
            ).joinpath("images")
        base_image_path.mkdir(parents=True, exist_ok=True) 
        self.image_path = base_image_path.joinpath(
            "hsv_cam_img.png"
        )
        self.raw_image_path = base_image_path.joinpath(
            "raw_img.jpg"
        )

        self.get_logger().info(
            f"\nProcessing the current view, please wait..."
        )

        self.image_capture_future = Future()

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/color/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.img_count = 0
        self.img_count_until_capture = 5
        self.waiting_for_image = True 

    def camera_callback(self, img_data): 
        cvbridge_interface = CvBridge() 
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            ) 
        except CvBridgeError as e:
            self.get_logger().warning(f"{e}")

        if self.img_count < self.img_count_until_capture:
            self.img_count += 1
        else: 
            self.waiting_for_image = False
            self.proc_image(img=cv_img)  
            self.image_capture_future.set_result('done')        

    def proc_image(self, img):
        cv2.imwrite(str(self.raw_image_path), img)

        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        pixel_colors = rgb_img.reshape((np.shape(rgb_img)[0]*np.shape(rgb_img)[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        pixel_colors = norm(pixel_colors).tolist()

        hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

        h, s, v = cv2.split(hsv_img)
        
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].scatter(h.flatten(), s.flatten(), facecolors=pixel_colors, marker=".")
        ax[0].set_xlabel("Hue")
        ax[0].set_ylabel("Saturation")
        ax[0].grid(True)
        
        ax[1].imshow(rgb_img)
        ax[1].axis("off")

        plt.tight_layout()
        fig.savefig(self.image_path)
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    rclpy.spin_until_future_complete(
        node, node.image_capture_future
    ) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    