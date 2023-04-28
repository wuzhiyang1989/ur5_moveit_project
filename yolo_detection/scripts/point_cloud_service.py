#!/home/ts/anaconda3/envs/YOLO/bin/python3

import cv2
import time
import rospy

from yolo_detection.srv import *
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

def get_point(req):
    print("ina")
    msg = rospy.wait_for_message("/kinect2/hd/points", PointCloud2, timeout=5)
    points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans = False, uvs = [(req.u, req.v)])

    resp   = GetPointResponse()
    print("@@@@@",resp)
    resp.position.x = points[0].x
    resp.position.y = points[0].y
    resp.position.z = points[0].z

    return resp


def point_cloud_service():
    rospy.init_node('point_cloud_service')

    s = rospy.Service('point_cloud', GetPoint, get_point)
    print("PointCloud service is ready...")
    rospy.spin()


if __name__ == "__main__":
    point_cloud_service()