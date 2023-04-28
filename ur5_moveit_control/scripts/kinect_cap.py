#!/home/ts/anaconda3/envs/YOLO/bin/python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

row    = 305
colu   = 250
height = 505
weigth = 1350

index  = 0

rospy.init_node('kinect_cap', anonymous=True)
rospy.loginfo("Robot  initializing")

while True:
    img_msg = rospy.wait_for_message('/kinect2/hd/image_color', Image, timeout=None)
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    img_crop = cv_img[row:row+height, colu:colu+weigth]
    row_crop = img_crop.shape[0]
    print(row_crop)
    col_crop = img_crop.shape[1]
    print(col_crop)
    img_pad = np.pad(img_crop, ((0,col_crop - row_crop),(0,0),(0,0)),"constant", constant_values=185)

    cv2.imshow("Kinect_cap_pad", img_pad)
    key = cv2.waitKey(3)
    if key == ord('s') or key == ord('S'):
        cv2.imwrite('./' + str(index) + '.png', img_pad)
        rospy.loginfo("Save image: ./" + str(index) + ".png")
        index += 1
    if key == ord('q') or key == ord('Q'):
        break