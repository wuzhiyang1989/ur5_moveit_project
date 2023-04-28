#!/home/ts/anaconda3/envs/YOLO/bin/python3


import sys

import cv2
import rospy
import tf
import tf2_ros
# from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
# from tf2_geometry_msgs import tf2_geometry_msgs

from tf_tools.srv import *
from yolo_detection.msg import *
from yolo_detection.srv import *
from dh_usb_control.msg import DHCmd


class InteractiveInterface(object):
    """docstring for InteractiveInterface"""
    def __init__(self):
        super(InteractiveInterface, self).__init__()
        # rospy.init_node('Ts', anonymous=True)
        rospy.loginfo(sys.argv)

    def image_client(self):
        pub = rospy.Publisher('image_cmd', String, queue_size=10) #10
        cmd_str = "Take an iamge."
        while True:
            try:
                pub.publish(cmd_str)
                data = rospy.wait_for_message("save_state", String, timeout=2)
                print("获取图片")
                print(data)
                return data
            except:
                continue
                print("Save image Error: Timeout!!!")
                return False


    def target_client(self, image_name):
        rospy.wait_for_service('yolo_target')
        try:
            target = rospy.ServiceProxy('yolo_target', Target)
            resp = target(image_name)
            print(resp)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None


    def object_client(self, image_name):
        rospy.wait_for_service('yolo_object')
        try:
            obj  = rospy.ServiceProxy('yolo_object', Task)
            resp = obj(image_name)
            #print("detection：获取点云信息")
            print(resp)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None


    def get_point_client(self, columns, rows):
        rospy.wait_for_service('point_cloud')
        # print("ina")
        try:
            point_client = rospy.ServiceProxy('point_cloud', GetPoint)
            req = GetPointRequest()
            req.u = columns
            req.v = rows
            resp = point_client(req)
            print("detection：获取点云信息")
            print(resp)
            return(resp)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None


    # def translate(self, position_x, position_y, position_z):
    #     # buffer = tf2_ros.Buffer()
    #     # sub = tf2_ros.TransformListener(buffer)
    #     tf_buffer = tf2_ros.Buffer()
    #     tf_listener = tf2_ros.TransformListener(tf_buffer)

    #     #定义要转换的输入
    #     p = PointStamped()
    #     p.header.stamp = rospy.Time()
    #     p.header.frame_id = 'kinect2_rgb_optical_frame'
    #     p.point.x = position_x
    #     p.point.y = position_y
    #     p.point.z = position_z

    #     try:
    #         res = tf_buffer.transform(p,'base_link',timeout=rospy.Duration(5))
    #         print(res)
    #     except rospy.ServiceException as e:
    #         print("TF translate: %s"%e)

    def translate_service_client(self, position_x, position_y, position_z):
        rospy.wait_for_service('Translate')
        
        req = TranslateRequest()
        # print(position_x, position_y, position_z)
        # print(type(position_x))
        req.position.x = float(position_x)
        req.position.y = float(position_y)
        req.position.z = float(position_z)

        try:
            translate_client = rospy.ServiceProxy('Translate', Translate)
            resp = translate_client(req)
            print(resp)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
    
    def gripper_client(self, cmd, data):
        pub = rospy.Publisher('ts_hand_cmd', DHCmd, queue_size=10)
        cmd_msg = DHCmd()
        cmd_msg.cmd = cmd
        cmd_msg.data = data

        rate = rospy.Rate(10)
        index = 0
        while index < 5:
            pub.publish(cmd_msg)
            index = index + 1
            rate.sleep()
