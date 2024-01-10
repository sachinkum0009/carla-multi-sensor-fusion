#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
from ament_index_python.packages import get_package_share_directory

WIDTH = 1280
HEIGHT = 720
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'raw_image', 10)
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    package_share_directory = get_package_share_directory('sensor_fusion_perception')
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    # self.cap = cv2.VideoCapture('dc8b39b5-fd030e88.mov')
    self.get_logger().info(package_share_directory)
    self.cap = cv2.VideoCapture(package_share_directory+'/src/driving_-_800 (360p).mp4')
    if self.cap.isOpened():
      self.get_logger().info("opened the video file successfullly")
    else:
      self.get_logger().error("opened the video file unsuccessfullly")

         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
    # print(frame.shape)
          
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # new_frame = cv2.resize(frame, (int(WIDTH/2), int(HEIGHT/2)))
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
      # Display the message on the console
      self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()