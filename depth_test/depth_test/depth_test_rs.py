import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from matplotlib import pyplot as plt



class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.show_depth_image(depth_image)
            # depth_array = np.array(depth_image, dtype=np.float32)
            # Extracting distance (you might need to adjust this depending on the actual data)
            # distance = depth_array[100, 100]  # Example: getting distance at pixel (100, 100)
            # plt.imshow(depth_array)
            # plt.title("depth map")
            # plt.show()
            # self.get_logger().info(f'Distance at (100, 100): {distance} meters')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")


    def show_depth_image(self, depth_image):
        fig, ax = plt.subplots()
        ax.imshow(depth_image, cmap='jet')
        ax.set_title('Depth Image')
        ax.set_xlabel('Pixels')
        ax.set_ylabel('Pixels')
        fig.canvas.mpl_connect('button_press_event', lambda event: self.on_click(event, depth_image))
        plt.show()

    def on_click(self, event, depth_image):
        if event.inaxes is not None:
            x, y = int(event.xdata), int(event.ydata)
            if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                distance = depth_image[y, x]
                print(f"Distance at ({x}, {y}): {distance/1000.0} meters")

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
