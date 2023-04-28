#!/home/ts/anaconda3/envs/YOLO/bin/python3

import rospy
from yolo_detection.srv import *


def point_cloud_service_test(u, v):
    rospy.wait_for_service('point_cloud')
    try:
        point_client = rospy.ServiceProxy('point_cloud', GetPoint)
        req = GetPointRequest()
        req.u = u
        req.v = v
        resp = point_client(req)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Test Point Cloud Service...")
    print("Input image coordination.")

    rows = 411
    colu = 821
    point_cloud_service_test(colu, rows)