# %%
import torch
import urllib.request
import cv2

# %%
zoe = torch.hub.load("isl-org/ZoeDepth", "ZoeD_N", pretrained=True)
url, filename = ("https://github.com/max-mapper/cats/raw/master/cat_photos/12a8d742be7f11e188131231381b5c25_7.png", "12a8d742be7f11e188131231381b5c25_7.png")
urllib.request.urlretrieve(url, filename)
image = cv2.imread(filename)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# %%
predicted_depth = zoe.infer_pil(image, pad_input=False)  # Better 'metric' accuracy

