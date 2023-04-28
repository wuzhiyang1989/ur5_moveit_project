#! /usr/bin/env python

import rospy
import time

from tf_tools.srv import *

def translate_service_01_client(position_x, position_y, position_z):
# def translate_service_01_client():
    rospy.wait_for_service('Translate')
    
    req = TranslateRequest()
    # req.frame = "bottle"
    print(position_x, position_y, position_z)
    print(type(position_x))
    req.position.x = float(position_x)
    req.position.y = float(position_y)
    req.position.z = float(position_z)

    try:
        translate_client = rospy.ServiceProxy('Translate', Translate)
        resp = translate_client(req)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    while True:
        cmd = input("imput cmd:")
        if cmd == 'q' or cmd == 'Q':
            break
        # if cmd =='y' or cmd == "Y":
        else:
            x = input("input position_x:")
            y = input("input position_y:")
            z = input("input position_z:")
            print("Position: ", x, y, z)
            translate_service_01_client(x, y, z)