#!/usr/bin/env python3
#coding=utf-8

import argparse
import copy
import math
import os
import sys
import threading
import time
from math import pi

import actionlib
import moveit_commander
import numpy as np
import rospy
import tf
from controller_manager_msgs.srv import (SwitchController,
                                         SwitchControllerRequest,
                                         SwitchControllerResponse)
from franka_gripper.msg import GraspAction, GraspEpsilon, GraspGoal
from franka_msgs.srv import (SetCartesianImpedance,
                             SetCartesianImpedanceRequest,
                             SetCartesianImpedanceResponse)
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from tf_tools.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint

from hm_data import *


class MoveITControl:
    def __init__(self):
        #初始化ros节点 名为panda_grasp
        # rospy.init_node('panda_grasp', anonymous=True)
        # 初始化 MoveIT 控制API
        self.mvoeit_init()
        # 初始化夹爪 Ation client
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_client.wait_for_server()
        # 初始化 franka 任务控制命令的 subscriber 和 publisher
        rospy.Subscriber("franka_cmd", Int8MultiArray, self.callback)
        self.pub   = rospy.Publisher('franka_task', Int8MultiArray, queue_size=1)
        # 初始化任务参数
        self.run        = [-1, -1, -1]
        self.publish    = [-1, -1, -1]
        self.last_task  = [-1, -1, -1]
        self.next_task  = [-1, -1, -1]
        self.sub_thread = threading.Thread(target=self.spin_thread)
        self.sub_thread.start()
        self.pub_thread = threading.Thread(target=self.publish_thread)
        self.pub_thread.start()
        # 张开夹爪
        # self.set_gripper(0.078,epsilon=0.00)#张开8cm
        pass

    def mvoeit_init(self):
        # 初始化MoveIT API接口
        moveit_commander.roscpp_initialize(sys.argv)
        self.lock = threading.Lock()
        switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        cartesian_impedance_proxy = rospy.ServiceProxy('/franka_control/set_cartesian_impedance', SetCartesianImpedance)
        cartesian_impedance_proxy.wait_for_service()

        request = SetCartesianImpedanceRequest
        request.cartesian_impedance = [2000,500,500,50,50,50]
        con = SwitchControllerRequest()
        con.strictness= con.STRICT
        con.start_controllers =[]
        con.stop_controllers = ['position_joint_trajectory_controller']

        with self.lock:
            rospy.sleep(0.1)
            switcher(con)
            result = cartesian_impedance_proxy(request.cartesian_impedance)
            con.stop_controllers =[]
            con.start_controllers = ['position_joint_trajectory_controller']
            print(result)
            switcher(con)
        
        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)
        # 创建机械臂规划组对象
        self.panda_arm = moveit_commander.MoveGroupCommander('panda_arm')
        # 设置最大加速度以及最大角加速度
        self.set_scaling_factor(0.5)
        #通过此发布器发布规划的轨迹
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
        # 获取末端执行器名称
        self.end_effector_link = self.panda_arm.get_end_effector_link()
        rospy.loginfo("End effector detected {}".format(self.end_effector_link))         

        # 设置允许机械臂末位姿的错误余量
        self.panda_arm.set_goal_position_tolerance(0.01)#1cm
        self.panda_arm.set_goal_orientation_tolerance(0.05)#

        #不允许规划失败重规划,规划时间只允许5秒钟,否则很浪费时间
        self.panda_arm.allow_replanning(False)
        self.panda_arm.set_planning_time(5)
        pass

    def set_scaling_factor(self, factor):
        # 设置最大加速度以及最大角加速度
        self.panda_arm.set_max_acceleration_scaling_factor(factor)
        self.panda_arm.set_max_velocity_scaling_factor(factor)


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
            msg.data = self.publish
            self.pub.publish(msg)
            rate.sleep()

    def set_gripper(self,gripper_width,epsilon=0.0):
        '''
        #设置panda 夹爪的开合大小
        #gripper_width 最大0.08m
        '''
        if gripper_width>0.08 or gripper_width<0.0:
            raise Exception
        #帮助维持夹爪力度
        grasp_epsilon = GraspEpsilon(epsilon,epsilon)
        goal = GraspGoal(width = gripper_width, speed = 0.08,epsilon=grasp_epsilon ,force=3.0)
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def cartesian_move(self, pose, execute = True):
        self.panda_arm.set_start_state_to_current_state()  #以当前姿态作为规划起始点
        # waypoints = self.waypoints_caculate(pose)
        waypoints = pose
        # 规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
        (plan, fraction) = self.panda_arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
            ##显示轨迹
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.panda_arm.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # 执行,并等待这个轨迹执行成功
        new_plan=self.scale_trajectory_speed(plan, 0.3)
        print("Wait for action..")
        # time.sleep(5)
        if execute:
            self.panda_arm.execute(new_plan, wait=True)


    # def waypoints_caculate(self, pose):
    #     waypoints = pose
    #     # wpose = pose[0]
    #     return waypoints
    #     pass


    def check_aruco(self):
        try:
            data  = rospy.wait_for_message(product_topic[self.run[2]], Pose, timeout=3)
        except:
            print("Don't receive data in 3 secends.")
            return -1
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
            print(resp)
            return [resp.translated_position.x, resp.translated_position.y, resp.translated_position.z]
            # return resp.translated_position
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        pass

    def get_position_goal(self, position, product, prepare, z = -1):
        goal = Pose()
        if prepare:
            goal.position.z = product_height[product] + 0.25 + 0.10
        else:
            goal.position.z = product_height[product] + 0.25 + 0.01
        if z != -1:
            goal.position.z = z
        # 处理放置物品时，放置在订单的左边还是右边

        goal.position.x = position[0]
        goal.position.y = position[1]

        goal.orientation.x = orientation_1[0]
        goal.orientation.y = orientation_1[1]
        goal.orientation.z = orientation_1[2]
        goal.orientation.w = orientation_1[3]
        return goal
        pass

    def scale_trajectory_speed(self,traj,spd=0.1):
        new_traj = RobotTrajectory()
        new_traj = traj

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)

        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd

            points[i] = point

        new_traj.joint_trajectory.points = points     
        return   new_traj

    def sort_to_order(self):
        self.set_scaling_factor(0.4)
        t1 = time.time()
        print("franka: wait for product aruco pose.")
        position_goal = self.check_aruco()    # return type: geometry_msgs/Point
        if position_goal == -1:
            return 0

        sort = self.run[0] - 12
        if sort > 2:
            rospy.loginfo(position_goal)
            position_goal[1] = position_goal[1]-0.01
            rospy.loginfo(position_goal)
        w_points = []

        print(f"Franka: append sort {sort} prepare pose.")
        goal_position = self.get_position_goal(position_goal[0:2], self.run[2], True)
        w_points.append(copy.deepcopy(goal_position))

        print(f"Franka: append sort {sort} pose.")
        if self.run[2] == 0:
            position_goal[2] = position_goal[2]+0.03
        
        goal_position = self.get_position_goal(position_goal[0:2], self.run[2], False, z=position_goal[2]+0.09)
        w_points.append(copy.deepcopy(goal_position))

        print("Franka: execute waypoints.")
        self.cartesian_move(w_points)
        print("Franka: close gripper.")
        self.set_gripper(0.05,epsilon=0.4)  # 张开3cm
        self.publish = self.run

        w_points.clear()
        print(f"Franka: append sort {sort} prepare pose.")
        goal_position = self.get_position_goal(position_goal[0:2], self.run[2], True)
        w_points.append(copy.deepcopy(goal_position))

        if self.run[1] > 17 and self.run[1] < 21:
            order = self.run[1]-18
        elif self.run[1] == 23: 
            order = 3
        else:
            return -1
        if order != 2:
            order_flag = self.check_order(order)
            if order_flag:
                order_position[order][1] = order_position[order][1] + 0.05
            else:
                order_position[order][1] = order_position[order][1] - 0.08
        print(f"Franka: move to order {order} prepare position.")
        goal_position = self.get_position_goal(order_position[order], self.run[2], True)
        w_points.append(copy.deepcopy(goal_position))

        print(f"Franka: move to order {order}.")
        if self.run[2] == 0:
            position_goal[2] = position_goal[2]+0.03
        goal_position = self.get_position_goal(order_position[order], self.run[2], False, z=position_goal[2]+0.11)
        w_points.append(copy.deepcopy(goal_position))

        print("Franka: execute waypoints.")
        self.cartesian_move(w_points)
        print("Franka: open gripper.")
        self.set_gripper(0.08,epsilon=0.4)  # 张开3cm

        w_points.clear()
        print(f"Franka: move to order {order} prepare position.")
        goal_position = self.get_position_goal(order_position[order], self.run[2], True)
        w_points.append(copy.deepcopy(goal_position))

        print("Franka: execute waypoints.")
        self.cartesian_move(w_points)
        self.panda_arm.go(init_joint, wait=True)
        t2 = time.time()
        print(f"T = t2 - t1 = {t2 - t1}")
        pass

    def shelf_to_sort(self):
        self.set_scaling_factor(0.4)
        print(f"Franka: move to prepare joint {self.run[0]}.")
        self.panda_arm.go(prepare_joint[self.run[0]], wait=True)

        self.set_scaling_factor(0.2)
        print(f"Franka: move to object joint {self.run[0]}.")
        self.panda_arm.go(object_joint[self.run[0]], wait=True)

        print("Franka: close gripper.")
        self.set_gripper(0.03,epsilon=0.3)

        print(f"Franka: move to prepare joint {self.run[0]}.")
        self.panda_arm.go(prepare_joint[self.run[0]])
        self.publish = self.run
        self.set_scaling_factor(0.4)

        print("Franka: move to init joint.")
        self.panda_arm.go(init_joint, wait=True)

        sort = self.run[1]-12
        w_points = []
        print(f"Franka: move to sort prepare position {sort}.")
        goal_position = self.get_position_goal(sort_position[sort], self.run[2], True)
        w_points.append(copy.deepcopy(goal_position))

        print(f"Franka: move to sort {sort}.")
        goal_position = self.get_position_goal(sort_position[sort], self.run[2], False)
        w_points.append(copy.deepcopy(goal_position))
        
        print("Franka: execute waypoints.")
        self.cartesian_move(w_points)

        print("Franka: open gripper.")
        self.set_gripper(0.08,epsilon=0.4)

        print("Franka: move to init joint.")
        self.panda_arm.go(init_joint, wait=True)
        print("Franka: task finished.")

        w_points.clear()
        pass

    def check_order(self, order):
        data = rospy.wait_for_message("/transfer_state", Int8MultiArray, timeout=3)
        data = np.array(data.data).reshape(10, 12)
        if np.any(data[6 + order] == 1):
            return True
        else:
            return False

    def check_cmd(self):
        self.sort = [[-0.463, 0.061],
                     [-0.264, 0.067],
                     [-0.050, 0.070],
                     [+0.174, 0.069]]
        topic_list = ["/aruco_simple/pose_601", "/aruco_simple/pose_602", "/aruco_simple/pose_603",
                      "/aruco_simple/pose_604", "/aruco_simple/pose_605", "/aruco_simple/pose_606",
                      "/aruco_simple/pose_607", "/aruco_simple/pose_608", "/aruco_simple/pose_609",
                      "/aruco_simple/pose_610", "/aruco_simple/pose_611", "/aruco_simple/pose_612"]
        data = rospy.wait_for_message(topic_list[self.run[2]], Pose, timeout=3)
        index = self.detection(data.position)
        if index == self.run[0]:
            return True
        else:
            return False
        
    def detection(self, position):
        if position.x > self.sort[0][0] and position.x < self.sort[1][0]:
            if position.y > self.sort[0][1]:
                return 14         # sort 3
            else:
                return 17         # sort 6
        if position.x > self.sort[1][0] and position.x < self.sort[2][0]:
            if position.y > self.sort[1][1]:
                return 13         # sort 2
            else:
                return 16         # sort 5
        if position.x > self.sort[2][0] and position.x < self.sort[3][0]:
            if position.y > self.sort[2][1]:
                return 12         # sort 1
            else:
                return 15         # sort 4
        # if position.x > self.order[0][0] and position.y < self.order[0][1]:
        #     return 20             # trash
        # if position.x > self.order[2][0] and position.y > self.order[2][1]:
        #     return 21             # order 1
        # if position.y > self.order[1][1] and position.y < self.order[2][1] and position.x > self.order[1][0]:
        #     return 18             # order 2
        # if position.y > self.order[0][1] and position.y < self.order[1][1] and position.x > self.order[1][0]:
        #     return 19             # order 3
        return -1
def main():
    rospy.init_node('franka_control', anonymous=True)
    robot = MoveITControl()
    # robot.panda_arm.go(init_joint, wait=True)

    # rate = rospy.Rate(0.5)
    print("Franka: the task will start in 5 seconds.")
    time.sleep(5)
    while not rospy.is_shutdown():
        # robot.run = robot.next_task
        # print(robot.run)
        if not (robot.run == [-1, -1, -1]):  # 判定 robot.run 为可执行任务
            print(f"Franka: Execute task >> {robot.run}.")
            if (robot.run[0] >= 0) and (robot.run[0] < 12):
                robot.shelf_to_sort()
                pass
            if (robot.run[0] > 11) and (robot.run[0] < 18):
                if robot.check_cmd():
                    robot.sort_to_order()
                    pass
            robot.last_task = robot.run
            robot.run       = [-1, -1, -1]
            robot.publish   = [-1, -1, -1]
        if robot.last_task == robot.next_task:
            # print("Franka: There is no executable task.")
            pass
            # robot.run = [-1, -1, -1]
        else:
            print("Franka: Get next task.")
            robot.run = robot.next_task
        # rate.sleep()
        # break
    # print("Process finished.")

if __name__ == '__main__':
    main()