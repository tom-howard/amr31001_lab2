#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.task import Future
from ament_index_python.packages import get_package_share_directory

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 

from pathlib import Path 
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

class ImageCapture(Node):

    def __init__(self): 
        super().__init__("object_detection")

        self.get_logger().info(
            f"\nSaving an image, please wait..."
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
            self.save_image(img=cv_img, img_name="cam_img")  
            self.image_capture_future.set_result('done')        

    def save_image(self, img, img_name): 

        base_image_path = Path(
            get_package_share_directory("amr31001_lab2")
            ).joinpath("images")
        base_image_path.mkdir(parents=True, exist_ok=True) 
        self.full_image_path = base_image_path.joinpath(
            f"{img_name}.jpg") 

        cv2.imwrite(str(self.full_image_path), img) 
    
    def proc_image(self, img): 
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        pixel_colors = rgb_img.reshape((np.shape(rgb_img)[0]*np.shape(rgb_img)[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        pixel_colors = norm(pixel_colors).tolist()

        hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

        h, s, v = cv2.split(hsv_img)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.scatter(h.flatten(), s.flatten(), facecolors=pixel_colors, marker=".")
        ax.set_xlabel("Hue")
        ax.set_ylabel("Saturation")
        ax.grid(True)
        fig.savefig(Path(str(self.full_image_path).replace(".jpg", "_hsv.png")))

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
    