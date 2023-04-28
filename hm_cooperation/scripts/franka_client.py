#!/usr/bin/env python3

import sys
import time
import rospy
from ocrtoc_common.srv import *
from ocrtoc_common.gripper_interface import GripperInterface

joint_point1 = [1.842458, -2.137526, 0.013304, -1.327917, -1.626963, -0.018947]

init_joint   = [-0.0318784608867538, -1.1706948260942116, -0.0235811203038641, 
                -2.61693128920438  , 0.00020199109533648,  1.5272900696595506, 0.779742430658804   ]

def joint_space_goal_client(joint_goal):
	try:
		joint_space_goal =rospy.ServiceProxy('send_joint_space_goal', JointSpaceGoal)
		responce = joint_space_goal(joint_goal)
		return responce

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def usage():
	return "%s joint_goal"%sys.argv[0]

if __name__ == '__main__':

	rospy.init_node('test_transform_interface')


	# Joint 控制
	print("Franka run to Joint point 1-1.")
	resp = joint_space_goal_client(init_joint)
	rospy.sleep(3.0)
	print(resp.successed)
	print(resp.text)

