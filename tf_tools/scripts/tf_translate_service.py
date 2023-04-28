#! /usr/bin/env python3
# coding=UTF-8

import rospy
import time
import tf2_ros
# import tf
from geometry_msgs.msg import PoseStamped,PointStamped, TransformStamped
from tf2_geometry_msgs import PoseStamped,PointStamped
from tf_tools.srv import *

def translate_service(req):
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)

    # 定义要转换的输入
    p = PointStamped()
    p.header.stamp = rospy.Time()
    p.header.frame_id = 'kinect2_rgb_optical_frame'
    p.point.x = req.position.x
    p.point.y = req.position.y
    p.point.z = req.position.z

    res = buffer.transform(p,'base_link',timeout=rospy.Duration(5))
    print(res)
    resp = TranslateResponse()
    resp.translated_position.x = res.point.x
    resp.translated_position.y = res.point.y
    resp.translated_position.z = res.point.z
    return resp
    

if __name__=="__main__":

    rospy.init_node("positon_traslate")

    s = rospy.Service('Translate', Translate, translate_service)
    
    rospy.spin()