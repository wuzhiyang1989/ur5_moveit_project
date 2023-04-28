#!/home/ts/anaconda3/envs/YOLO/bin/python3

import rospy
import torch
from YOLO.pick.yolo import YOLO
from yolo_detection.srv import *

    
def yolo_server():
    rospy.init_node('object_server')
    yolo = YOLO()
    s = rospy.Service('yolo_object', Task, yolo.predict)
    rospy.spin()
if __name__ == "__main__":
    yolo_server()