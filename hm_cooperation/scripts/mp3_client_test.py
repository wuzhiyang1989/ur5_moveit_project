#!/home/wy/anaconda3/envs/daily/bin/python3
import sys
import rospy
from hm_cooperation.srv import *

def mp3_client(cmd):
    rospy.wait_for_service('Mp3_player')
    req = Mp3Request()
    req.cmd = cmd
    print(cmd)
    try:
        mp3_client = rospy.ServiceProxy('Mp3_player', Mp3)
        resp = mp3_client(req)
        print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    while True:
        from_p  = int(input("Input cmd_from:"))
        to_p    = int(input("Input cmd_to:"))
        product = int(input("Input cmd_product:"))
        mp3_client([from_p, to_p, product])
        key = input("Continue?")
        if key == 'q' or key == 'Q':
            break