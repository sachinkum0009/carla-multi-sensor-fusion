# %%
# import libraries
from nuscenes.utils.data_classes import RadarPointCloud, view_points
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import numpy as np
# %%
# load data
pc = RadarPointCloud.from_file('n008-2018-08-01-15-16-36-0400__RADAR_FRONT__1533151603555991.pcd')

# %%
points = pc.points[:3, :]

# %%
# prepare model
model = DBSCAN(eps=3, min_samples=5)

# %%
points = points.T
points.shape

# %%
X = np.array([[1,2], [2,2], [2, 3], [8, 7], [8, 8], [25, 80]])
X.shape
# %%
# prepare model
model = DBSCAN(eps=4, min_samples=2)

model.fit(points)
labels = model.labels_
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print("Estimated number of clusters: %d" % n_clusters_)
print("Estimated number of nose points: %d" % n_noise_)

# %%
unique_labels = set(labels)
core_samples_mask = np.zeros_like(labels, dtype=bool)
core_samples_mask[model.core_sample_indices_] = True

colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = labels == k

    xy = points[class_member_mask & core_samples_mask]

    plt.plot(
        xy[:, 0],
        xy[:, 1],
        "o",
        markerfacecolor=tuple(col),
        markeredgecolor="k",
        markersize=14,
    )

    xy = points[class_member_mask & ~core_samples_mask]
    plt.plot(
        xy[:, 0],
        xy[:, 1],
        "o",
        markerfacecolor=tuple(col),
        markeredgecolor="k",
        markersize=6,
    )

plt.title(f"Estimated number of clusters: {n_clusters_}")
plt.show()
# %%
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pc.points[0, :], pc.points[1, :], pc.points[2, :])
plt.show()

# %%
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
unique_labels = set(labels)
core_samples_mask = np.zeros_like(labels, dtype=bool)
core_samples_mask[model.core_sample_indices_] = True

colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = labels == k

    xy = points[class_member_mask & core_samples_mask]

    ax.scatter(
        xy[:, 0],
        xy[:, 1],
        xy[:, 2],
        "o",
        color=tuple(col),
        edgecolors="k",
        s=14,
    )

    xy = points[class_member_mask & ~core_samples_mask]
    ax.scatter(
        xy[:, 0],
        xy[:, 1],
        xy[:, 2],
        "o",
        color=tuple(col),
        edgecolors="k",
        s=6,
    )

plt.title(f"Estimated number of clusters: {n_clusters_}")
plt.show()