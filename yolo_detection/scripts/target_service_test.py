#!/home/ts/anaconda3/envs/YOLO/bin/python

import sys
import rospy
import numpy
import torch

from yolo_detection.msg import *
from yolo_detection.srv import *

def target_service_test(image_name):
    rospy.wait_for_service('yolo_target')
    try:
        target = rospy.ServiceProxy('yolo_target', Target)
        resp = target(image_name)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def object_service_test(image_name):
    rospy.wait_for_service('yolo_object')
    try:
        obj  = rospy.ServiceProxy('yolo_object', Task)
        resp = obj(image_name)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("test yolo target detect dervice...")
    # print("image: /home/ts/Pictures/ts_ws/detect_crop.png")
    index =1
    while True:
        # cmd = input("?")
        image_name = str(index) + ".png"
    # target_service_test(image_name)
        object_service_test(image_name)
        index += 1
