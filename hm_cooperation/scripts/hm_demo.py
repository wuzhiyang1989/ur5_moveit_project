#!/usr/bin/env python
import rospy
import time

from ocrtoc_common.srv import *
from tf_tools.srv import *
from ocrtoc_common.gripper_interface import GripperInterface
from geometry_msgs.msg import Pose, Point

#***********************************************************************
# hm_cooperation class:
#   pub task to franka robot
#***********************************************************************
class hm_cooperation(object):
    """docstring for hm_cooperation"""
    def __init__(self):
        super(hm_cooperation, self).__init__()

        self.gripper = GripperInterface(topic_name = '/franka_gripper/gripper_action')
        self.sort_list = [601, 602, 603, 604, 605, 606]

        try:
            rospy.wait_for_service('send_joint_space_goal')
            self.joint_space_goal =rospy.ServiceProxy('send_joint_space_goal', JointSpaceGoal)

            rospy.wait_for_service('/send_pose_goal')
            self.service_call = rospy.ServiceProxy('/send_pose_goal', PoseGoal)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def get_state_client(self):
        rospy.wait_for_service('get_manipulator_state')
        try:
            state_service_call =rospy.ServiceProxy('get_manipulator_state', ManipulatorState)
            responce = state_service_call()
            return responce

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def joint_space_goal_client(self, joint_goal):
        rospy.wait_for_service('send_joint_space_goal')
        try:
            joint_space_goal =rospy.ServiceProxy('send_joint_space_goal', JointSpaceGoal)
            responce = joint_space_goal(joint_goal)
            return responce

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def get_position_goal(self, position_goal):
        goal = PoseGoalRequest()
        goal.goal.position.x = position_goal[0]
        goal.goal.position.y = position_goal[1]
        goal.goal.position.z = position_goal[2]
        goal.goal.orientation.x = position_goal[3]
        goal.goal.orientation.y = position_goal[4]
        goal.goal.orientation.z = position_goal[5]
        goal.goal.orientation.w = position_goal[6]
        return goal


    def position_goal_client(self, position_goal):       #position_goal type: geometry_msgs/Pose
        request = self.get_position_goal(position_goal)
        rospy.wait_for_service('send_pose_goal')
        try:
            pose_service_call =rospy.ServiceProxy('send_pose_goal', PoseGoal)
            responce = pose_service_call(request)
            return responce

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def shelves_to_sort(self, goods_num, sort_num):
        self.joint_space_goal_client(prepare_joint[goods_num-1])
        self.joint_space_goal_client(goods_joint[goods_num-1])
        '''
        闭合夹爪
        '''
        self.gripper.close()

        self.joint_space_goal_client(prepare_joint[goods_num-1])
        self.joint_space_goal_client(sort_joint[sort_num-1])
        '''
        打开夹爪
        '''
        self.gripper.open()

        self.joint_space_goal_client(init_joint)


    def sort_to_order(self, sort_num, order_num):
        goods_position = self.check_aruco(sort_num)
        self.position_goal_client([goods_position.translated_position.x, goods_position.translated_position.y, goods_position.translated_position.z+0.15, 
            gripper_orientation[0], gripper_orientation[1], gripper_orientation[2], gripper_orientation[3]])
        self.position_goal_client([goods_position.translated_position.x, goods_position.translated_position.y, goods_position.translated_position.z+0.07, 
            gripper_orientation[0], gripper_orientation[1], gripper_orientation[2], gripper_orientation[3]])

        '''
        闭合夹爪
        '''
        self.gripper.close()
        self.position_goal_client([goods_position.translated_position.x, goods_position.translated_position.y, goods_position.translated_position.z+0.2, 
            gripper_orientation[0], gripper_orientation[1], gripper_orientation[2], gripper_orientation[3]])

        self.joint_space_goal_client(order_joint[order_num-1])
        '''
        打开夹爪
        '''
        self.gripper.open()

        self.joint_space_goal_client(init_joint)


    def check_aruco(self, sort_num):
        aruco = self.sort_list[sort_num]
        aruco = 2
        data  = rospy.wait_for_message(topic_list[aruco-1], Pose, timeout=10)

        rospy.wait_for_service('Translate')
        req = TranslateRequest()
        req.frame = "goods"
        req.position.x = float(data.position.x)
        req.position.y = float(data.position.y)
        req.position.z = float(data.position.z)

        try:
            translate_client = rospy.ServiceProxy('Translate', Translate)
            resp = translate_client(req)
            # print(resp)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)




#***********************************************************************
# main function
#***********************************************************************
if __name__ == "__main__":
    rospy.init_node('franka_control', anonymous=True)
    
    hm = hm_cooperation()
    time.sleep(5)

    # hm.shelves_to_sort(2, 3)
    # hm.sort_to_order(4, 3)