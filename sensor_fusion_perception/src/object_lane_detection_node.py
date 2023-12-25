import argparse
import os, sys
import shutil
import time
from pathlib import Path
import imageio


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory
package_share_directory = get_package_share_directory('sensor_fusion_perception')

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

print(sys.path)
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import scipy.special
import numpy as np
import torchvision.transforms as transforms
import PIL.Image as image

from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result
from lib.core.function import AverageMeter
from lib.core.postprocess import morphological_process, connect_lane
from tqdm import tqdm
normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

class ObjectLaneDetection(Node):
    def __init__(self):
        super().__init__('object_lane_detection_node')
        self.initialize_detection_pipeline()
        self.bridge = CvBridge()
        self.img_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.img_subscriber = self.create_subscription(Image, 'raw_image', self.image_callback, 10)

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            self.get_logger().error("not able to convert the image to cv")

        self.detect_object(cv_img)

        
    def initialize_detection_pipeline(self):
        logger, _, _ = create_logger(
        cfg, cfg.LOG_DIR, 'demo')

        self.device = select_device(logger,'0')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load('weights/End-to-end.pth', map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.device)
        if self.half:
            self.model.half()  # to FP16


        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

        # Run inference

        img = torch.zeros((1, 3, 640, 640), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        self.model.eval()


    def detect_object(self, img):
        old_img = img
        img_det = img
        img = cv2.resize(img, (640, 384))

        img = transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        with torch.no_grad():
            det_out, da_seg_out,ll_seg_out= self.model(img)
            torch.cuda.empty_cache()

        inf_out, _ = det_out

        # Apply NMS
        det_pred = non_max_suppression(inf_out, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False)

        det=det_pred[0]

        _, _, height, width = img.shape
        pad_w = 0
        pad_h = 12
        ratio = 0.5

        da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
        da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
        # da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)

        
        ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
        # Lane line post-processing
        #ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
        #ll_seg_mask = connect_lane(ll_seg_mask)

        img_det = show_seg_result(img_det, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

        if len(det):
            det[:,:4] = scale_coords(img.shape[2:],det[:,:4],img_det.shape).round()
            for *xyxy,conf,cls in reversed(det):
                label_det_pred = f'{self.names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, img_det , label=label_det_pred, color=self.colors[int(cls)], line_thickness=2)
        
        # cv2.imshow('image', img_det)
        # cv2.waitKey(0)  # 1 millisecond

        try:
            ros_img = self.bridge.cv2_to_imgmsg(img_det, encoding='bgr8')
            self.img_publisher.publish(ros_img)
            self.get_logger().info("publishing image")
        except CvBridgeError:
            self.get_logger().error("not able to convert the cv2 to ros msg")
        




def main(args=None):
    rclpy.init(args=args)
    # try:
    node = ObjectLaneDetection()
    rclpy.spin(node)
    # finally:
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
