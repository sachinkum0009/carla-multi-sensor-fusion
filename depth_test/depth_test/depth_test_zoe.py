import torch
from time import time

repo = "isl-org/ZoeDepth"
# Zoe_N
model_zoe_n = torch.hub.load(repo, "ZoeD_N", pretrained=True)

##### sample prediction
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
zoe = model_zoe_n.to(DEVICE)


# Local file
from PIL import Image
image = Image.open("outdoor_image1.png").convert("RGB")  # load

start_time = time()
depth_numpy = zoe.infer_pil(image)  # as numpy
estimated_time = time() - start_time
print("estimated time is {} seconds".format(estimated_time))

start_time = time()
depth_numpy = zoe.infer_pil(image)  # as numpy
estimated_time = time() - start_time
print("estimated time is {} seconds".format(estimated_time))

start_time = time()
depth_numpy = zoe.infer_pil(image)  # as numpy
estimated_time = time() - start_time
print("estimated time is {} seconds".format(estimated_time))

start_time = time()
depth_numpy = zoe.infer_pil(image)  # as numpy
estimated_time = time() - start_time
print("estimated time is {} seconds".format(estimated_time))

from matplotlib import pyplot as plt
def on_click(event, depth_image):
        if event.inaxes is not None:
            x, y = int(event.xdata), int(event.ydata)
            if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                distance = depth_image[y, x]
                print(f"Distance at ({x}, {y}): {distance} meters")

# plt.imshow(depth_numpy)
# plt.title("depth map")
# plt.show()

fig, ax = plt.subplots()
ax.imshow(depth_numpy, cmap='jet')
ax.set_title('Depth Image')
ax.set_xlabel('Pixels')
ax.set_ylabel('Pixels')
fig.canvas.mpl_connect('button_press_event', lambda event: on_click(event, depth_numpy))
plt.show()