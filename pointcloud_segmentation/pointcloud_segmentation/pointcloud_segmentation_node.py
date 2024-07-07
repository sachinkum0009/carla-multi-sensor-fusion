#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from sklearn.cluster import DBSCAN
import numpy as np
import time
from matplotlib import pyplot as plt


class PointCloudSegmentationNode(Node):
    def __init__(self):
        super().__init__("pointcloud_segmentation_node")
        self.img_sub = self.create_subscription(PointCloud2, "/velodyne_points", self.pointcloud_cb, 10)
        self.pointcloud_segment_pub = self.create_publisher(PointCloud2, "pointcloud_semantic_points", 10)
        self.db_scan = DBSCAN(eps=0.4, min_samples=20)
        self.get_logger().info("node started")

    def pointcloud_cb(self, msg: PointCloud2):
        self.get_logger().info("received point cloud data")
        # pts = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        pts = self.pointcloud2_to_array(msg)
        
        colors = self.segment_pointcloud(pts)
        self.get_logger().info("clustered point cloud data")
        self.array_to_pointcloud2(pts, colors)
        
    def pointcloud2_to_array(self, cloud_msg):
        points = pc2.read_points(cloud_msg, field_names=["x", "y", "z"], skip_nans=True)
        return np.vstack([points['x'], points['y'], points['z']])

    def segment_pointcloud(self, points: np.ndarray) -> np.ndarray:
        self.db_scan.fit(points.T)
        labels = self.db_scan.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        print("Estimated number of clusters: %d" % n_clusters_)
        print("Estimated number of noise points: %d" % n_noise_)

        # Define colors for each cluster label
        colors = np.zeros((len(labels), 1))  # Initialize colors array
        unique_labels = set(labels)
        for i, label in enumerate(unique_labels):
            if label == -1:
                colors[labels == label] = [0]  # Noise points in black
            else:
                colors[labels == label] = label # plt.cm.Spectral(i / len(unique_labels))[:1]  # Assign color based on label

        return colors
    
    def array_to_pointcloud2(self, points: np.ndarray, colors: np.ndarray) -> PointCloud2:
        stamp = self.get_clock().now().to_msg()
        frame_id = 'base_scan'
        # pc = pc2.create_cloud_xyz32(pc2.Header(stamp=stamp, frame_id=frame_id), points)

        header = pc2.Header()
        header.frame_id = frame_id
        header.stamp = stamp

        # print(colors.T.shape)
        # print(points.shape)

        points = np.vstack([points, colors.T])

        # print(points.shape)
        # print(len(points))

        fields =[pc2.PointField(name='x',  offset=0, datatype=pc2.PointField.FLOAT32, count = 1),
                pc2.PointField(name='y',  offset=4, datatype=pc2.PointField.FLOAT32, count = 1),
                pc2.PointField(name='z',  offset=8, datatype=pc2.PointField.FLOAT32, count = 1),
                pc2.PointField(name='intensity',  offset=12, datatype=pc2.PointField.FLOAT32, count = 1)
                ]
        pc = pc2.create_cloud(header, fields, points.T.astype(np.float32) )
        self.pointcloud_segment_pub.publish(pc)



def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()