#!/usr/bin/env python3

import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
from nuscenes.utils.data_classes import RadarPointCloud, view_points
from sensor_msgs_py import point_cloud2
import ros2_numpy as rnp


class CameraAndRadarPublisher(Node):
    def __init__(self):
        super().__init__("camera_and_radar_publisher")

        # variables
        path = get_package_share_directory("sensor_fusion_perception")
        self.get_logger().info(path)
        self.img_dir = os.path.join(path, "datasets/images/CAM_FRONT/")
        self.radar_dir = os.path.join(path, "datasets/radar_points/RADAR_FRONT/")
        self.get_logger().info(self.img_dir)


        images = os.listdir(self.img_dir)
        radar_points = os.listdir(self.radar_dir)
        self.num_of_files = len(images)
        self.i = 0

        self.images_and_radar_points = list(zip(images, radar_points))
        # print(self.images_and_radar_points)
        # exit(1)
        # cv bridge
        self.bridge = CvBridge()

        # publisher and subscriptions
        self.image_publisher = self.create_publisher(Image, "image_raw", 1)
        self.radar_publisher = self.create_publisher(PointCloud2, "radar", 1)
        # timer
        self.create_timer(1, self.timer_callback)


    def numpy_to_pointcloud2(self, points, frame_id='base_link', stamp=None):
        

        # Define fields
        # fields = [
        #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        # ]

        # Create a generator expression to iterate over points and yield each x, y, z tuple
        data = np.zeros(points.shape[1], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
        ])
        data['x'] = points[0, :]
        data['y'] = points[1, :]
        data['z'] = points[2, :]
        # points_generator = ((float(p[0]), float(p[1]), float(p[2])) for p in points)

        msg = rnp.msgify(PointCloud2, data)
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        # Create PointCloud2 message
        # msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp

        # # Pack the points into a binary buffer
        # data = np.fromiter(points_generator, dtype=np.float32, count=len(points) * 3)

        # # Reshape the data to a 2D array with 3 columns
        # data = data.reshape((len(points), 3))

        # # Convert the numpy array to a binary buffer
        # data_buffer = data.tobytes()

        # # Set the height, width, and fields in the PointCloud2 message
        # msg.height = 1
        # msg.width = len(points)
        # msg.fields = fields
        # msg.is_bigendian = False
        # msg.point_step = 12  # 4 bytes for each x, y, z
        # msg.row_step = msg.point_step * msg.width
        # msg.is_dense = True  # Assumes there are no invalid points in the cloud
        # msg.data = data_buffer

        return msg

    def timer_callback(self):
        if self.i >= self.num_of_files:
            self.i = 0

        # self.get_logger().info(os.path.join(self.img_dir, self.images_and_radar_points[self.i][0]))
        # read the image using opencv
        img = cv2.imread(os.path.join(self.img_dir, self.images_and_radar_points[self.i][0]))
        print(img.shape)
        # convert to ros2 using bridge
        try:
            ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # publish ros Image
            self.image_publisher.publish(ros_img)
        except CvBridgeError as e:
            self.get_logger().error("Error converted  %s" % e)

        # self.get_logger().info(os.path.join(self.radar_dir, self.images_and_radar_points[self.i][1]))
        # read radar pointcloud
        pc = RadarPointCloud.from_file(os.path.join(self.radar_dir, self.images_and_radar_points[self.i][1]))
        # Points live in the point sensor frame. So they need to be transformed via global to the image plane.
        # First step: transform the pointcloud to the ego vehicle frame for the timestamp of the sweep.
        # cs_record = self.nusc.get('calibrated_sensor', pointsensor['calibrated_sensor_token'])
        cs_record = {'token': 'f4d2a6c281f34a7eb8bb033d82321f79', 'sensor_token': '47fcd48f71d75e0da5c8c1704a9bfe0a', 'translation': [3.412, 0.0, 0.5], 'rotation': [0.9999984769132877, 0.0, 0.0, 0.0017453283658983088], 'camera_intrinsic': []}
        pc.rotate(Quaternion(cs_record['rotation']).rotation_matrix)
        pc.translate(np.array(cs_record['translation']))

        # Second step: transform from ego to the global frame.
        # poserecord = self.nusc.get('ego_pose', pointsensor['ego_pose_token'])
        poserecord1 = {'token': 'b70cefb08263499eb30c7e7da0031428', 'timestamp': 1532402928114656, 'rotation': [0.5742482223921863, -0.00183950588546914, 0.014160690493351492, -0.8185566994058888], 'translation': [409.8439847252494, 1176.9519903782593, 0.0]}
        pc.rotate(Quaternion(poserecord1['rotation']).rotation_matrix)
        pc.translate(np.array(poserecord1['translation']))

        # Third step: transform from global into the ego vehicle frame for the timestamp of the image.
        # poserecord = self.nusc.get('ego_pose', cam['ego_pose_token'])
        poserecord2 = {'token': '4b6870ae200c4b969b91c50a9737f712', 'timestamp': 1532402928112460, 'rotation': [0.5742377826385253, -0.0018667925555496464, 0.014165885989800115, -0.8185638715152707], 'translation': [409.8506686425138, 1176.9702106041582, 0.0]}
        pc.translate(-np.array(poserecord2['translation']))
        pc.rotate(Quaternion(poserecord2['rotation']).rotation_matrix.T)

        # Fourth step: transform from ego into the camera.
        # cs_record = self.nusc.get('calibrated_sensor', cam['calibrated_sensor_token'])
        cs_record = {'token': '1d31c729b073425e8e0202c5c6e66ee1', 'sensor_token': '725903f5b62f56118f4094b46a4470d8', 'translation': [1.70079118954, 0.0159456324149, 1.51095763913], 'rotation': [0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755], 'camera_intrinsic': [[1266.417203046554, 0.0, 816.2670197447984], [0.0, 1266.417203046554, 491.50706579294757], [0.0, 0.0, 1.0]]}
        pc.translate(-np.array(cs_record['translation']))
        pc.rotate(Quaternion(cs_record['rotation']).rotation_matrix.T)

        # Fifth step: actually take a "picture" of the point cloud.
        # Grab the depths (camera frame z axis points away from the camera).
        # depths = pc.points[2, :]
        # print(pc.points)

        pointcloud_msg = self.numpy_to_pointcloud2(pc.points, frame_id="front_radar")

        self.radar_publisher.publish(pointcloud_msg)

        

        # print('depth', depths)
        # retreive the color from depth
        # coloring = depths

        # Take the actual picture (matrix multiplication with camera-matrix + renormalization).
        # points = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=True)
        # print(points.shape)

        # Remove points that are either outside or behind the camera. Leave a margin of 1 pixel for aesthetic reasons.
        # Also make sure points are at least 1m in front of the camera to avoid seeing the lidar points on the camera
        # casing for non-keyframes which are slightly out of sync.
        # min_dist: float = 1.0
        # dot_size: int = 5
        # mask = np.ones(depths.shape[0], dtype=bool)
        # mask = np.logical_and(mask, depths > min_dist)
        # mask = np.logical_and(mask, points[0, :] > 1)
        # mask = np.logical_and(mask, points[0, :] < im.size[0] - 1)
        # mask = np.logical_and(mask, points[1, :] > 1)
        # mask = np.logical_and(mask, points[1, :] < im.size[1] - 1)
        # points = points[:, mask]
        # coloring = coloring[mask]

        # new_points = points[:2, :]

        # all_points = np.vstack([new_points, coloring]).T

        self.i += 1

        # print(pc.points[0])


def main(args=None):
    rclpy.init(args=args)
    camera_and_radar_publisher_node = CameraAndRadarPublisher()
    rclpy.spin(camera_and_radar_publisher_node)
    camera_and_radar_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
