import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from nuscenes.utils.data_classes import RadarPointCloud, view_points
from pyquaternion import Quaternion
from sklearn.cluster import DBSCAN
from PIL import Image, ImageDraw, ImageFont
from copy import deepcopy
from matplotlib import animation

from sort import Sort

# Directory containing the files
directory = '/media/asus/backup/Nuscenes/data/sets/nuscenes/samples'

# Filter files based on keywords
image_files = [file for file in os.listdir(os.path.join(directory, 'CAM_FRONT'))]
pcd_files = [file for file in os.listdir(os.path.join(directory, 'RADAR_FRONT'))]

# Sort files by their timestamp assuming filename format: 'prefix_timestamp.extension'
image_files.sort(key=lambda x: int(x.split('_')[-1].split('.')[0]))
pcd_files.sort(key=lambda x: int(x.split('_')[-1].split('.')[0]))

# Choose a font (you need to provide the path to a font file)
font = ImageFont.truetype("arial.ttf", 36)

# Determine the position to draw the text (in this example, at the top-left corner)
position = (10, 10)

# Define the color of the text
text_color = (255, 0, 0)  # white


i = 0
dbscan = DBSCAN(eps=1, min_samples=2)
# instance of sort
mot_tracker = Sort(max_age=5, min_hits=1, iou_threshold=0.5) 


fig, ax = plt.subplots(1, 1, figsize=(9, 16))
fig.canvas.set_window_title("sample token")



def process_radar(current_pc : RadarPointCloud, previous_pc: RadarPointCloud, im: Image):
    # global previous_pc
    print("earlier prev", previous_pc.points.shape)
    print("earlier cur", current_pc.points.shape)
    # step 1: check if previous frame radar data is available
    # step 2: combine the points from the previous and current frame of radar data
    points_xyz = np.concatenate((previous_pc.points[0:3, :].T, current_pc.points[0:3, :].T), axis=0)
    # print("points xyz", points_xyz[:, 1:3])
    # step 3: db scan to find the clusters

    output = dbscan.fit(points_xyz[:, 1:3])
    labels = output.labels_
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of nose points: %d" % n_noise_)

    # step 4: find the cluster detections
    # Abscissa is horizontal axis in a two-dimensional coordinate system, often denoted as the x-axis
    # Ordinate is vertical axis in a two-dimensional coordinate system, often denoted as the y-axis
    # Find abscissa and ordinate for each cluster
    dets_list = []
    for cluster_id in set(output.labels_):
        if cluster_id != -1:  # -1 represents noise points
            cluster_points = [point for point, label in zip(points_xyz, output.labels_) if label == cluster_id]
            x_min, x_max = min(point[0] for point in cluster_points), max(point[0] for point in cluster_points)
            # y_min, y_max = min(point[2] for point in cluster_points), max(point[2] for point in cluster_points)
            z_avg = sum(point[2] for point in cluster_points)
            # print("z_avg", z_avg)
            y_min, y_max = z_avg - 2, z_avg + 2
            # for point in cluster_points:
            #     print(point[2])
            # # z_avg = z_avg/len(cluster_points[2])
            # print(len(cluster_points))
            # print(f"Cluster {cluster_id}: Abscissa ({x_min}, {x_max}), Ordinate ({y_min}, {y_max}), Z Avg ({z_avg})")
            dets_list.append([x_min, y_min, x_max, y_max])
    
    print(dets_list)

    if (len(dets_list)) != 0:

        dets_np = np.array(dets_list)
        print("dets np shape", dets_np.shape)
        trackers = mot_tracker.update(dets_np)
        print("len of trackers", len(trackers))


    else:
        trackers = []
    im_draw = ImageDraw.Draw(im)
    im_draw.text(position, "number of trackers: " + str(len(trackers)), fill=text_color, font=font)

    

    # for d in trackers:
    #     print('%d,%.2f,%.2f,%.2f,%.2f,1,-1,-1,-1'%(d[4],d[0],d[1],d[2]-d[0],d[3]-d[1]))
    
    # step 5: getting the core points from the clusters
    # Extract indices of core points (non-noise points)
    core_points_mask = np.zeros_like(output.labels_, dtype=bool)
    core_points_mask[output.core_sample_indices_] = True

    # Extract noise points
    noise_points_mask = output.labels_ == -1

    # Create a new variable without noise points
    cleaned_data = points_xyz[core_points_mask]

    # Optionally, you can store the noise points in a separate variable
    noise_data = points_xyz[noise_points_mask]

    # Print the cleaned data and noise points
    # print("Cleaned Data:")
    # print(cleaned_data)
    # print("\nNoise Points:")
    # print(noise_data)
    cleaned_data_pc = current_pc
    cleaned_data_pc.points = cleaned_data.T
    cleaned_data_points, cleaned_data_coloring = calibrate_radar_points(cleaned_data_pc, im)
    # %matplotlib inline
    ax.clear()
    ax.imshow(im)
    ax.scatter(cleaned_data_points[0, :], cleaned_data_points[1, :], c=cleaned_data_coloring, s=5)
    ax.axis('off')
    # ani = FuncAnimation(fig, update, frames=range(num_frames), interval=100)
    # plt.show()
    # plt.pause(0.01)  # Adjust the pause time as needed
    previous_pc.points = current_pc.points
    points_xyz = np.array([])
    print("previous_pc ", previous_pc.points.shape)
    print("point xyz", points_xyz.shape)
    # input("waiting for next frame")


def apply_transformation(pc: RadarPointCloud) -> RadarPointCloud:
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
    return pc

def calibrate_radar_points(pc: RadarPointCloud, im: Image) -> tuple[np.ndarray, np.ndarray]:
    # Points live in the point sensor frame. So they need to be transformed via global to the image plane.
    # First step: transform the pointcloud to the ego vehicle frame for the timestamp of the sweep.
    # cs_record = self.nusc.get('calibrated_sensor', pointsensor['calibrated_sensor_token'])
    cs_record = {'token': '1d31c729b073425e8e0202c5c6e66ee1', 'sensor_token': '725903f5b62f56118f4094b46a4470d8', 'translation': [1.70079118954, 0.0159456324149, 1.51095763913], 'rotation': [0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755], 'camera_intrinsic': [[1266.417203046554, 0.0, 816.2670197447984], [0.0, 1266.417203046554, 491.50706579294757], [0.0, 0.0, 1.0]]}
    

    # Fifth step: actually take a "picture" of the point cloud.
    # Grab the depths (camera frame z axis points away from the camera).
    depths = pc.points[2, :]
    # print('depth', depths)
    # retreive the color from depth
    coloring = depths

    # Take the actual picture (matrix multiplication with camera-matrix + renormalization).
    points_with_depth = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=False)
    points = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=True)
    # print(points.shape)
    # print(points_with_depth.shape)
    # for i in range(10):
    #     print('x: ', int(points.T[i][0]), ' y: ', int(points.T[i][1]), ' z: ', int(points.T[i][2]))

    # print('-----')
    # for i in range(10):
    #     print('x: ', int(points_with_depth.T[i][0]), ' y: ', int(points_with_depth.T[i][1]), ' z: ', int(points_with_depth.T[i][2]))


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
    return points, coloring


# for current_index in range(1, len(image_files) - 1):
def animate(current_index):
    current_image_file = image_files[current_index]
    current_pcd_file = pcd_files[current_index]
    
    next_image_file = image_files[current_index + 1]
    next_pcd_file = pcd_files[current_index + 1]
    
    current_image_path = os.path.join(directory, 'CAM_FRONT', current_image_file)
    current_pcd_path = os.path.join(directory, 'RADAR_FRONT', current_pcd_file)
    
    next_image_path = os.path.join(directory, 'CAM_FRONT', next_image_file)
    next_pcd_path = os.path.join(directory, 'RADAR_FRONT', next_pcd_file)
    
    # Read current frame image
    current_image = Image.open(current_image_path)
    
    # Read current frame PCD
    current_pcd = apply_transformation(RadarPointCloud.from_file(current_pcd_path))
    
    # Read next frame image
    next_image = Image.open(next_image_path)
    
    # Read next frame PCD
    next_pcd = apply_transformation(RadarPointCloud.from_file(next_pcd_path))

    process_radar(next_pcd, current_pcd, next_image)

    # i+=1
    # if i > 100:
    #     break

ani = animation.FuncAnimation(fig, animate, interval=200)
plt.show()

'''
iou value = intersection area / union area

'''