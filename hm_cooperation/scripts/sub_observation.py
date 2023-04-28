#!/home/wy/anaconda3/envs/daily/bin/python3

import rospy
from hm_cooperation.msg import Observe

def test_observation(data):
    print(data)

if __name__ == '__main__':
    rospy.init_node("test_sub_observation")
    rospy.Subscriber("/Observation", Observe, test_observation)
    rospy.spin()