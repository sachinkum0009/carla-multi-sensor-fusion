import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt

class DepthRGBSubscriber(Node):
    def __init__(self):
        super().__init__('depth_rgb_subscriber')
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.rgb_callback,
            10)
        self.subscription_depth  # prevent unused variable warning
        self.subscription_rgb  # prevent unused variable warning
        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def rgb_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image = rgb_image
            self.show_rgb_image(rgb_image)
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def on_click(self, event):
        if event.inaxes is not None and self.depth_image is not None:
            x, y = int(event.xdata), int(event.ydata)
            if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                distance = self.depth_image[y, x]
                print(f"Distance at ({x}, {y}): {distance/1000.0} meters")

    def show_rgb_image(self, rgb_image):
        fig, ax = plt.subplots()
        ax.imshow(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
        ax.set_title('RGB Image')
        ax.set_xlabel('Pixels')
        ax.set_ylabel('Pixels')
        fig.canvas.mpl_connect('button_press_event', lambda event: self.on_click(event))
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    depth_rgb_subscriber = DepthRGBSubscriber()
    rclpy.spin(depth_rgb_subscriber)
    depth_rgb_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
