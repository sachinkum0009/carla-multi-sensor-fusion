#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import time

class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__("semantic_segmentation_node")
        self.img_sub = self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.semantic_img_pub = self.create_publisher(Image, "semantic_img", 10)
        self.bridge = CvBridge()

    def img2cv(self, img: Image) -> np.ndarray:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            return cv_img
        
        except CvBridgeError as e:
            self.get_logger().error("Could not convert img msg to cv2 img")
            return np.ndarray((100, 100))
        
    def cv2img(self, cv: np.ndarray) -> Image:
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv, encoding='bgr8')
            return img_msg
        
        except CvBridgeError as e:
            self.get_logger().error("Could not convert cv2 img to img msg")
            return np.ndarray((100, 100))


    def image_cb(self, msg: Image):
        cv_img = self.img2cv(msg)
        img_msg = self.cv2img(cv_img)
        self.get_logger().info("publishing the image")
        self.semantic_img_pub.publish(img_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()