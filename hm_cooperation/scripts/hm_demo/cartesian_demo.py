#!/usr/bin/env python3
#coding=utf-8

import argparse
import copy
import math
import os
import sys
import time
from math import pi
from threading import Event, Lock

import actionlib
import moveit_commander
import numpy as np
# from sqlalchemy import false
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
# from gpg.msg import GraspConfig, GraspConfigList
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_from_matrix, quaternion_matrix,
                                quaternion_multiply)
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItDemo:
    def __init__(self):
        # 初始化MoveIT API接口
        moveit_commander.roscpp_initialize(sys.argv)
        #初始化ros节点 名为panda_grasp
        rospy.init_node('panda_grasp', anonymous=True)

        self.lock = Lock()
        switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        cartesian_impedance_proxy = rospy.ServiceProxy('/franka_control/set_cartesian_impedance', SetCartesianImpedance)
        cartesian_impedance_proxy.wait_for_service()

        request = SetCartesianImpedanceRequest
        request.cartesian_impedance = [2000,500,500,50,50,50]
        con = SwitchControllerRequest()
        #con.start_controllers = 'position_joint_trajectory_controller'
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
        # Franka Gripper Action Client
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_client.wait_for_server()

        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.sleep(2)
        # 创建机械臂规划组对象
        self.panda_arm = moveit_commander.MoveGroupCommander('panda_arm')
        #创建机械手规划对象
        self.panda_hand=moveit_commander.MoveGroupCommander('hand')
        # 设置最大加速度以及最大角加速度
        self.panda_arm.set_max_acceleration_scaling_factor(0.1)
        self.panda_arm.set_max_velocity_scaling_factor(0.1)
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
        # 张开夹爪
        self.set_gripper(0.078,epsilon=0.01)#张开8cm


    def cartesian_demo(self):
        self.panda_arm.set_start_state_to_current_state()  #以当前姿态作为规划起始点
        waypoints = []
        wpose=self.panda_arm.get_current_pose().pose
        wpose.position.x += 0.2
        wpose.position.z -= 0.2
        waypoints.append(copy.deepcopy(wpose))

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
        new_plan=self.scale_trajectory_speed(plan,0.3)
        print("Wait for action..")
        time.sleep(5)
        self.panda_arm.execute(new_plan,wait=True)

        # 执行抓取
        rospy.loginfo("Start to grasp")
        self.set_gripper(0.05,epsilon=0.4)  # 张开3cm
        rospy.sleep(1)


    def joint_demo(self):
        self.ready_joints = [1.544, -0.755, 0.190, -2.713, 0.149, 2.027, 0.799]
        # self.move_to_joints(self.panda_arm,self.ready_joints,tag="reday pose")
        self.panda_arm.go(self.ready_joints, wait=True)
        pass


    def move_to_joints(self,group,joints,tag="initial pose"):
        #先从Initial 移动到HOME
        case,plan  = self.planJointGoal(group,joints)#返回真  就是找到轨迹    
        if case==2:
            rospy.loginfo("Move to {}".format(tag))
            # group.execute(plan,wait=True)
            group.go(plan,wait=True)
        elif case==1:
            rospy.loginfo("Already at {}".format(tag))

        else:
            raise SystemError("Home pose  trajectory  not found")

    def planJointGoal(self,movegroup,joint_goal,lable='Next'):
        current_joint = movegroup.get_current_joint_values()
        dis_pose =np.linalg.norm(np.array(joint_goal)-np.array(current_joint))
        #print(current_joint)
        #print(joint_goal)
        if dis_pose<0.008:
            return 1,None #已经到位
        else:
            movegroup.set_joint_value_target(joint_goal)
            plan = movegroup.plan()
            print(f"plan = {plan}")
            print(type(plan[1]))
            # if not plan.joint_trajectory.points:
            #     return 0,plan
            # else:#执行规划
            #     return 2,plan
            if not plan[1].joint_trajectory.points:
                return 0,plan
            else:#执行规划
                return 2,plan

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


    def set_gripper(self,gripper_width,epsilon=0.0):
        '''
        #设置panda 夹爪的开合大小
        #gripper_width 最大0.08m
        '''
        if gripper_width>0.08 or gripper_width<0.0:
            raise Exception
        #帮助维持夹爪力度
        grasp_epsilon = GraspEpsilon(epsilon,epsilon)
        goal = GraspGoal(width = gripper_width, speed = 0.08,epsilon=grasp_epsilon ,force=5.0)
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

        # rospy.loginfo("Gripper action completed")


if __name__ == "__main__":
    try:
        demo = MoveItDemo()
        # demo.cartesian_demo()
        # time.sleep(10)
        demo.joint_demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")