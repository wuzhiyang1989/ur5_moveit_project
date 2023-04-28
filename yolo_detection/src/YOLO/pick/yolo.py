import argparse
import os
import platform
import shutil
import time
from pathlib import Path

import rospy
import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn
# from numpy import random
from yolo_detection.msg import *
from yolo_detection.srv import *

import tf2_ros

# from tf2_geometry_msgs import PoseStamped,PointStamped

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from sensor_msgs import point_cloud2
# from sensor_msgs.msg import PointCloud2

from YOLO.pick.models.experimental import attempt_load
# from YOLO.pick.utils.datasets import LoadStreams, LoadImages
from YOLO.pick.utils.datasets import letterbox
from YOLO.pick.utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_labels,
    xyxy2xywh, plot_one_rotated_box, strip_optimizer, set_logging, rotate_non_max_suppression)
from YOLO.pick.utils.torch_utils import select_device, load_classifier, time_synchronized
from YOLO.pick.utils.evaluation_utils import rbox2txt

class YOLO(object):
    _defaults = {
        #--------------------------------------------------------------------------#
        #   初始化参数
        #
        #--------------------------------------------------------------------------#
        "weights"           : '/home/ts/catkin_ws/src/yolo_detection/src/YOLO/pick/best.pt',
        "source"            : '',
        "output"            : '/home/ts/Documents/YOLO/detection/',
        "imgsz"             : 608,  
        "conf_thres"        : 0.5,
        "iou_thres"         : 0.45,
        "device"            : '0,1',                                   #default='0,1', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        "view_img"          : True,
        "save_img"          : True,
        "save-txt"          : False,
        "classes"           : None,
        "agnostic_nms"      : False,
        "augment"           : False,
        "update"            : False,
        # 图像裁剪参数 
        "point_x"           : 300,
        "point_y"           : 250,
        "height"            : 1350,
        "weigth"            : 500,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
        set_logging()

        self.generate()

        # Get names and colors
        # 获取类别名字    names = ['person', 'bicycle', 'car',...,'toothbrush']
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        # 设置画框的颜色    colors = [[178, 63, 143], [25, 184, 176], [238, 152, 129],....,[235, 137, 120]]随机设置RGB颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

        # Run inference
        t0 = time.time()
        # 进行一次前向推理,测试程序是否正常  向量维度（1，3，imgsz，imgsz）
        # img = torch.zeros((1, 3, self.imgsz, self.imgsz), device=device)  # init img
        # _ = model(img.half() if self.half else img) if device.type != 'cpu' else None  # run once


    #---------------------------------------------------#
    #   生成模型
    #---------------------------------------------------#
    def generate(self):
        #---------------------------------------------------#
        #   建立yolo模型，载入yolo模型的权重
        #---------------------------------------------------#
        device = select_device(self.device)
        
        # 如果设备为gpu，使用Float16
        self.half = device.type != 'cpu'  # half precision only supported on CUDA

        self.model = attempt_load(self.weights, map_location=device)  # load FP32 model
        self.imgsz = check_img_size(self.imgsz, s=self.model.stride.max())  # check img_size

        # 设置Float16
        if self.half:
            self.model.half()  # to FP16


    #---------------------------------------------------#
    #   图片预测
    #---------------------------------------------------#
    def detect_image(self, image_name):
        device = select_device(self.device)
        # dataset = LoadImages(image_filename, img_size=self.imgsz)
        resp = []

        # for path, img, im0s, vid_cap in dataset:
        #-------------------------------------------------#
        #   调整 dataset 获取方式，不建立数据集，直接从 ros image message 加载
        #-------------------------------------------------#
        img_msg = rospy.wait_for_message('/kinect2/hd/image_color', Image, timeout=None)
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_crop = cv_img[self.point_x:self.point_x+self.weigth, self.point_y:self.point_y+self.height]
        row = img_crop.shape[0]
        col = img_crop.shape[1]
        # print(row, col)
        im0s = np.pad(img_crop, ((0,col-row),(0,0),(0,0)),"constant", constant_values=255)
        # print(img.shape)

        # Padded resize
        img = letterbox(im0s, new_shape=self.imgsz)[0]
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)        

        save_path = self.output + image_name
        cv2.imwrite(self.output + '1_' + image_name, img)

        img = torch.from_numpy(img).to(device)
        print(img.shape)
        # 图片也设置为Float16
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        # 没有batch_size的话则在最前面添加一个轴
        if img.ndimension() == 3:
            # (in_channels,size1,size2) to (1,in_channels,img_height,img_weight)
            img = img.unsqueeze(0)  # 在[0]维增加一个维度
        print("开始预测")

        pred = self.model(img, augment=self.augment)[0]
        pred = rotate_non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms, without_iouthres=False)

        # Process detections
        for i, det in enumerate(pred):  # i:image index  det:(num_nms_boxes, [xylsθ,conf,classid]) θ∈[0,179]
            s, im0 = '', im0s

            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            # print("det = ", det)
            if det is None:
                return None
            if det is not None and len(det):
                print("处理识别结果...")
                # Rescale boxes from img_size to im0 size
                det[:, :5] = scale_labels(img.shape[2:], det[:, :5], im0.shape).round()

                # Print results    det:(num_nms_boxes, [xylsθ,conf,classid]) θ∈[0,179]
                for c in det[:, -1].unique():  # unique函数去除其中重复的元素，并按元素（类别）由大到小返回一个新的无元素重复的元组或者列表
                    n = (det[:, -1] == c).sum()  # detections per class  每个类别检测出来的素含量
                    s += '%g %ss, ' % (n, self.names[int(c)])  # add to string 输出‘数量 类别,’

                for *rbox, conf, cls in reversed(det):  # 翻转list的排列结果,改为类别由小到大的排列

                    if self.save_img or self.view_img:  # Add bbox to image
                        label = '%s %.2f' % (self.names[int(cls)], conf)
                        classname = '%s' % self.names[int(cls)]
                        conf_str = '%.3f' % conf
                        plot_one_rotated_box(rbox, im0, label=label, color=self.colors[int(cls)], line_thickness=1,
                                                pi_format=False)

                        obj = Object()
                        obj.conf  = float('%.3f' % conf)
                        obj.label = classname
                        obj.angle = int(180-rbox[-1])
                        obj.row   = int(rbox[1])
                        obj.colu  = int(rbox[0])

                        resp.append(obj)
                        print("obj = ",obj)

            if self.save_img:
                cv2.imwrite(save_path, im0)
                # cv2.waitKey(0)
                pass
        return resp


    #---------------------------------------------------#
    #   目标检测service
    #---------------------------------------------------#
    def predict(self, req):
        # print("object detect begin!")

        obj_list = self.detect_image(req.image_filename)
        print("@@@@@@@@:",obj_list)
        
        resp = TaskResponse()
        if obj_list is not None:
            resp.detect_flag = True
            resp.obj = obj_list
        else:
            resp.detect_flag = False

        return resp
