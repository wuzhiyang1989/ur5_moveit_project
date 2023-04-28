#!/usr/bin/python3
import threading
import time

import actionlib
import control_msgs.msg
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from ocrtoc_common.srv import *
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from tf_tools.srv import *

from hm_data import *

# from ocrtoc_common.gripper_interface import GripperInterface

class GripperInterface(object):
    """Python gripper interface

    Args:
        topic_name(str): the ros topic name for the gripper action
    """
    def __init__(self, topic_name = '/franka_gripper/gripper_action'):
        self.topic_name = topic_name
        self._gripper_client = actionlib.SimpleActionClient(self.topic_name, control_msgs.msg.GripperCommandAction)

    def go_to_position(self, position, max_effort = 30, wait_time = 2.0):
        """Move the gripper to position

        Args:
            position(float): the target distance between the two fingers
            max_effort(float): max effort
            wait_time(float): time to wait after sending the command.
        """
        self._gripper_client.wait_for_server()
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = position / 2.0 # The topic target is the distance from one finger to the center.
        goal.command.max_effort = max_effort

        self._gripper_client.send_goal(goal)
        rospy.sleep(wait_time)

    def close(self):
        self.go_to_position(position = 0.0, max_effort = 10, wait_time = 2.0)

    def open(self):
        self.go_to_position(position = 0.078, max_effort = 30, wait_time = 2.0)


class control(object):
    """docstring for control"""
    def __init__(self):
        rospy.Subscriber("franka_cmd", Int8MultiArray, self.callback)
        self.pub   = rospy.Publisher('franka_task', Int8MultiArray, queue_size=1)

        # self.run        = [-1, -1, -1]
        self.run        = [15, 19,  1]
        self.next_task  = [-1, -1, -1]
        self.sub_thread = threading.Thread(target=self.spin_thread)
        self.sub_thread.start()
        self.pub_thread = threading.Thread(target=self.publish_thread)
        self.pub_thread.start()

        # init franka & gripper service and subscriber
        self.gripper = GripperInterface(topic_name = '/franka_gripper/gripper_action')
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


    def get_position_goal(self, position, product, prepare, z=-1):
        goal = PoseGoalRequest()
        if prepare:
            goal.goal.position.z = product_height[product] + 0.25 +0.2
        else:
            goal.goal.position.z = product_height[product] + 0.25 +0.01
        if not (z == -1):
            goal.goal.position.z = z

        goal.goal.position.x = position[0]
        goal.goal.position.y = position[1]
        goal.goal.orientation.x = orientation_1[0]
        goal.goal.orientation.y = orientation_1[1]
        goal.goal.orientation.z = orientation_1[2]
        goal.goal.orientation.w = orientation_1[3]
        return goal


    def position_goal_client(self, position_goal):       #position_goal type: geometry_msgs/Pose
        # request = self.get_position_goal(position_goal)
        rospy.wait_for_service('send_pose_goal')
        try:
            pose_service_call =rospy.ServiceProxy('send_pose_goal', PoseGoal)
            responce = pose_service_call(position_goal)
            return responce

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def check_aruco(self):
        data  = rospy.wait_for_message(product_topic[self.run[2]], Pose, timeout=10)
        print("Franka: wait for \"Translate\" service...")
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
            return [resp.translated_position.x, resp.translated_position.y, resp.translated_position.z]
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def get_order_position(self, position, product, prepare, z = -1, which_obj = None):
        goal = PoseGoalRequest()
        if prepare:
            goal.goal.position.z = product_height[product] + 0.25 +0.2
        else:
            goal.goal.position.z = product_height[product] + 0.25 +0.03
        if z != -1:
            goal.goal.position.z = z
        # 处理放置物品时，放置在订单的左边还是右边

        goal.goal.position.x = position[0]
        goal.goal.position.y = position[1]

        goal.goal.orientation.x = orientation_1[0]
        goal.goal.orientation.y = orientation_1[1]
        goal.goal.orientation.z = orientation_1[2]
        goal.goal.orientation.w = orientation_1[3]
        return goal
        pass


    def callback(self, msg):
        self.next_task = msg.data


    def spin_thread(self):
        rospy.spin()


    def publish_thread(self):
        msg  = Int8MultiArray()
        dim_msg  = MultiArrayDimension()
        dim_msg.size   = 3
        dim_msg.stride = 3
        dim_msg.label  = "cmd"
        msg.layout.dim.append(dim_msg)


        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            msg.data = self.run
            self.pub.publish(msg)
            rate.sleep()

'''
    def _async_raise(tid, exctype):
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def stop_thread(thread):
        _async_raise(thread.ident, SystemExit)
'''

def main():
    rospy.init_node('franka_control', anonymous=True)
    robot = control()
    robot.joint_space_goal_client(init_joint)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # robot.run = robot.next_task
        print(robot.run)
        if not (robot.run == [-1, -1, -1]):  # 判定 robot.run 为可执行任务
            print("Franka: the task will start in 5 seconds.")
            time.sleep(5)
            if (robot.run[0] >= 0) and (robot.run[0] < 12):
            # 货架到分拣区
                print(f"Franka: move to prepare joint {robot.run[0]}.")
                robot.joint_space_goal_client(prepare_joint[robot.run[0]])

                print(f"Franka: move to object joint {robot.run[0]}.")
                robot.joint_space_goal_client(object_joint[robot.run[0]])

                # print("Franka: close gripper.")
                # robot.gripper.close()

                print(f"Franka: move to prepare joint {robot.run[0]}.")
                robot.joint_space_goal_client(prepare_joint[robot.run[0]])

                print("Franka: move to init joint.")
                robot.joint_space_goal_client(init_joint)

                sort = robot.run[1]-12
                print(f"Franka: move to sort prepare position {sort}.")
                # get_position_goal(self, position, product, prepare):
                goal_position = robot.get_position_goal(sort_position[sort], robot.run[2], True)
                res = robot.position_goal_client(goal_position)
                print(res.successed)

                print(f"Franka: move to sort {sort}.")
                goal_position = robot.get_position_goal(sort_position[sort], robot.run[2], False)
                robot.position_goal_client(goal_position)
                
                print("Franka: open gripper.")
                robot.gripper.open()

                print("Franka: move to init joint.")
                flag = robot.joint_space_goal_client(init_joint)
                if flag.successed == True:
                    print("Franka： task finished.")

            if (robot.run[0] > 11) and (robot.run[0] < 18):
            # 分拣区到订单区或者回收站
                print("franka: wait for product aruco pose.")
                position_goal = robot.check_aruco()    # return type: geometry_msgs/Point
                # 补全 goal pose，抓取

                sort = robot.run[0] - 12
                print(f"Franka: move to sort prepare position {sort}.")
                goal_position = robot.get_position_goal(position_goal[0:2], robot.run[2], True)
                robot.position_goal_client(goal_position)

                print(f"Franka: move to sort {sort}.")
                goal_position = robot.get_position_goal(position_goal[0:2], robot.run[2], False, z=position_goal[2]+0.1)
                robot.position_goal_client(goal_position)

                # print("Franka: close gripper.")
                # robot.gripper.close()

                print(f"Franka: move to sort prepare position {sort}.")
                goal_position = robot.get_position_goal(position_goal[0:2], robot.run[2], True)
                robot.position_goal_client(goal_position)

                order = robot.run[1]-18
                print(f"Franka: move to order prepare position {order}.")
                goal_position = robot.get_order_position(order_position[order], robot.run[2], True)
                robot.position_goal_client(goal_position)

                print(f"Franka: move to order {order}.")
                goal_position = robot.get_order_position(order_position[order], robot.run[2], False, z=position_goal[2]+0.12)
                robot.position_goal_client(goal_position)

                # print("Franka: close gripper.")
                # robot.gripper.open()

                print("Franka: move to init joint.")
                flag = robot.joint_space_goal_client(init_joint)
                if flag.successed == True:
                    print("Franka： task finished.")

        else:
            print("Franka: wait a executable task...")
            continue
        # 任务执行完成之后将 self.run 置为[-1， -1， -1]
        robot.run = [-1, -1, -1]
        rate.sleep()

if __name__ == '__main__':
    main()
