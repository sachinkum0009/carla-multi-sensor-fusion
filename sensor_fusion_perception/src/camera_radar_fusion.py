# from nuscenes.nuscenes import NuScenes
from PIL import Image
# import open3d as o3d
from nuscenes.utils.data_classes import RadarPointCloud, view_points
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from sklearn.cluster import DBSCAN

# read image
im = Image.open('n008-2018-08-01-15-16-36-0400__CAM_FRONT__1533151603512404.jpg')

# plt.imshow(im)
# plt.axis('off')

# read pcd radar 
# pcd = o3d.io.read_point_cloud('n008-2018-08-01-15-16-36-0400__RADAR_FRONT__1533151603555991.pcd')
pc = RadarPointCloud.from_file('n008-2018-08-01-15-16-36-0400__RADAR_FRONT__1533151603555991.pcd')

# print(pc.points.shape)

# apply transform
# out_arr = np.asarray(pcd.points) 
# print("output array from input list : ", out_arr)
# plt.show()

# combine

# plot the image
# {'sensor_token': '47fcd48f71d75e0da5c8c1704a9bfe0a',
#   'translation': [3.412, 0.0, 0.5],
#   'rotation': [0.9999984769132877, 0.0, 0.0, 0.0017453283658983088]}

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pc.points[0, :], pc.points[1, :], pc.points[2, :])
plt.show()

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
depths = pc.points[2, :]
print('depth', depths)
# retreive the color from depth
coloring = depths

# Take the actual picture (matrix multiplication with camera-matrix + renormalization).
points_with_depth = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=False)
points = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=True)
print(points.shape)
print(points_with_depth.shape)
for i in range(10):
  print('x: ', int(points.T[i][0]), ' y: ', int(points.T[i][1]), ' z: ', int(points.T[i][2]))

print('-----')
for i in range(10):
  print('x: ', int(points_with_depth.T[i][0]), ' y: ', int(points_with_depth.T[i][1]), ' z: ', int(points_with_depth.T[i][2]))


# Remove points that are either outside or behind the camera. Leave a margin of 1 pixel for aesthetic reasons.
# Also make sure points are at least 1m in front of the camera to avoid seeing the lidar points on the camera
# casing for non-keyframes which are slightly out of sync.
min_dist : float = 1.0
dot_size : int = 5
mask = np.ones(depths.shape[0], dtype=bool)
mask = np.logical_and(mask, depths > min_dist)
mask = np.logical_and(mask, points[0, :] > 1)
mask = np.logical_and(mask, points[0, :] < im.size[0] - 1)
mask = np.logical_and(mask, points[1, :] > 1)
mask = np.logical_and(mask, points[1, :] < im.size[1] - 1)
points = points[:, mask]
coloring = coloring[mask]

# return points, coloring, im


# another function
# Init axes.
fig, ax = plt.subplots(1, 1, figsize=(9, 16))
fig.canvas.set_window_title("sample token")
ax.imshow(im)
ax.scatter(points[0, :], points[1, :], c=coloring, s=dot_size)
ax.axis('off')

# print(type(points))
# print(points.shape)
# shape is (3, 54)
for i in range(54):
  print('x: ', int(points.T[i][0]), ' y: ', int(points.T[i][1]), ' z: ', int(coloring[i]))

# print(points.shape)
# print('x: ', int(points.T[0][0]), ' y: ', int(points.T[0][1]), ' z: ', int(coloring[0]))

plt.show()

# print(pc.points[:3, :])
# print(len(pc.points[3, :]))