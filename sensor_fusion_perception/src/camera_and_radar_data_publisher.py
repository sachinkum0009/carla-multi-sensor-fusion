#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError


class CameraAndRadarPublisher(Node):
    def __init__(self):
        super().__init__("camera_and_radar_publisher")

        # variables
        path = get_package_share_directory("sensor_perception_fusion")
        img_dir = os.path.join(path, "/datasets/images/CAM_FRONT/")
        radar_dir = os.path.join(path, "/datasets/images/RADAR_FRONT/")

        images = os.listdir(img_dir)
        radar_points = os.listdir(radar_dir)
        self.num_of_files = len(images)
        self.i = 0

        self.images_and_radar_points = list(zip(images, radar_points))
        # cv bridge
        self.bridge = CvBridge()

        # publisher and subscriptions
        self.image_publisher = self.create_publisher(Image, "image_raw", 10)
        self.radar_publisher = self.create_publisher(PointCloud2, "radar", 10)
        # timer
        self.create_timer(0.0333, self.timer_callback)

    def timer_callback(self):
        if self.i < self.num_of_files:
            self.i = 0

        # read the image using opencv
        img = cv2.read_img(self.images_and_radar_points[0][self.i])
        # convert to ros2 using bridge
        try:
            ros_img = self.bridge.cv2_to_imgmsg(img, encoding="rgb")
            # publish ros Image
            self.image_publisher.publish(ros_img)
        except CvBridgeError as e:
            self.get_logger().error("Error converted  %s" % e)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    camera_and_radar_publisher_node = CameraAndRadarPublisher()
    rclpy.spin(camera_and_radar_publisher_node)
    camera_and_radar_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
