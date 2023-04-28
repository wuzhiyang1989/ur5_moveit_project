#!/home/ts/anaconda3/envs/YOLO/bin/python3
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

point_x = 300
point_y = 250
height  = 1350
weigth  = 500

def callback(data):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

    img_crop = cv_img[point_x:point_x+weigth, point_y:point_y+height]
    cv2.imshow("kinect v2" , img_crop)
    cv2.waitKey(3)
    pass
 
def displayROScam():
    rospy.init_node('cam_display_demo', anonymous=True)

    # 修改 topic 名称
    rospy.Subscriber('/kinect2/hd/image_color', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':
    displayROScam()
