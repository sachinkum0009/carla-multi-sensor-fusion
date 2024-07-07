#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import time

import open3d as o3d

class PointCloudSegmentationNode(Node):
    def __init__(self):
        super().__init__("pointcloud_segmentation_node")
        self.img_sub = self.create_subscription(PointCloud2, "/velodyne_points", self.pointcloud_cb, 10)
        self.pointcloud_segment_pub = self.create_publisher(PointCloud2, "pointcloud_semantic_points", 10)

    def pointcloud_cb(self, msg: PointCloud2):
        self.get_logger().info("received point cloud data")
        # pts = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        pts = self.pointcloud2_to_array(msg)
        
        clustered_pts = self.segment_pointcloud(list(pts))
        self.get_logger().info("clustered point cloud data")
        
    def pointcloud2_to_array(self, cloud_msg):
        points = list(pc2.read_points(cloud_msg, field_names=["x", "y", "z"], skip_nans=True))
        return np.array(points, dtype=np.float32)

    def segment_pointcloud(self, points):
        # Convert numpy array to Open3D point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        
        # Segment the point cloud using Euclidean clustering
        labels = np.array(cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
        
        # Separate the points into clusters
        max_label = labels.max()
        self.get_logger().info(f"point cloud segmented into {max_label + 1} clusters")
        
        # For simplicity, return points of the largest cluster
        largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
        largest_cluster_points = points[labels == largest_cluster_idx]
        
        return largest_cluster_points

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()