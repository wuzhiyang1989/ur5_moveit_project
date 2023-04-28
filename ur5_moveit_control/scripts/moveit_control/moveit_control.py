import rospy
# import rospy, sys
# import rospkg
# import rosparam
import moveit_commander
# import os

# import tf
# import argparse
# import math
import numpy as np
# from math import pi
# import time
import copy
# from threading import Lock, Event

# import actionlib

from moveit_msgs.msg import RobotTrajectory,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler,quaternion_multiply,quaternion_from_matrix,quaternion_matrix

# from gpg.msg import GraspConfig,GraspConfigList
# from autolab_core import RigidTransform

from moveit_control.ur5_position_data import *

class URMoveITControl:
    def __init__(self):
        # get_parameters()
        self.init_joint = init_joint
        # 创建机械臂规划组对象"
        self.robot_arm = moveit_commander.MoveGroupCommander("manipulator")
        # 设置机械臂最大速度以及加速度
        self.robot_arm.set_max_acceleration_scaling_factor(0.1)
        self.robot_arm.set_max_velocity_scaling_factor(0.1)
        # 通过此发布器发布规划的轨迹
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
        # 获取末端执行器名称
        self.end_effector_link = self.robot_arm.get_end_effector_link()
        rospy.loginfo("End effector detected {}".format(self.end_effector_link))         

        # 设置允许机械臂末位姿的错误余量
        self.robot_arm.set_goal_position_tolerance(0.01)#1cm
        self.robot_arm.set_goal_orientation_tolerance(0.05)#

        # 不允许规划失败重规划,规划时间只允许5秒钟,否则很浪费时间
        self.robot_arm.allow_replanning(False)
        self.robot_arm.set_planning_time(5)
        # rospy.sleep(5 )
        # 移动到工作姿态
        self.move_to_joints(self.robot_arm, self.init_joint,tag="working pose")
        # rospy.sleep(0.5)                          # 等待机械臂稳定

        # self.move_to_joints(self.robot_arm, hori_joint,tag="working pose")
        # rospy.sleep(0.5)

        # self.move_to_joints(self.robot_arm, vert_joint,tag="working pose")
        # rospy.sleep(0.5)

        # 测试笛卡尔坐标系
        # wpose = self.robot_arm.get_current_pose().pose
        # wpose.position.z -= 0.2
        # self.move_to_cartesian(copy.deepcopy(wpose))

        # test move EE
        # self.move_one_joint(self.robot_arm, 1.57, 6)
        # self.move_one_joint(self.robot_arm, -1.57, 6)
        # self.move_one_joint(self.robot_arm, -1.57, 6)
        # pass

    def planJointGoal(self,movegroup,joint_goal,lable='Next'):
        current_joint = movegroup.get_current_joint_values()
        dis_pose =np.linalg.norm(np.array(joint_goal)-np.array(current_joint))
        #print(current_joint)
        #print(joint_goal)
        if (dis_pose<0.008):
            return 1,None #已经到位
        else:
            movegroup.set_joint_value_target(joint_goal)
            # plan = movegroup.plan()
            plan_success, traj, planning_time, error_code = movegroup.plan()
            if not plan_success:
            # if not plan.joint_trajectory.points:
                return 0,traj
            else:#执行规划
                return 2,traj

    def move_one_joint(self, movegroup, rad, joint_num):
        current_joint = movegroup.get_current_joint_values()
        # print(current_joint, type(current_joint))
        if (rad<0.008) and (rad>-0.008):
            return 1,None #已经到位
        else:
            current_joint[joint_num-1] = current_joint[joint_num-1] + rad
            joint_goal = current_joint
            movegroup.set_joint_value_target(joint_goal)
            # plan = movegroup.plan()
            plan_success, traj, planning_time, error_code = movegroup.plan()
            if not plan_success:
            # if not plan.joint_trajectory.points:
                # return 0,traj
                pass
            else:#执行规划
                movegroup.execute(traj,wait=True)
                rospy.sleep(1)
                # return 2,traj
        pass

    def move_to_joints(self,group,joints,tag="initial pose"):
        #先从Initial 移动到HOME
        case,plan  = self.planJointGoal(group,joints)#返回真  就是找到轨迹    
        if case==2:
            # rospy.loginfo("Move to {}".format(tag))
            group.execute(plan,wait=True)
            rospy.sleep(1)
        elif case==1:
            rospy.loginfo("Already at {}".format(tag))

        else:
            raise SystemError("Home pose  trajectory  not found")
            

    def move_to_cartesian(self, pose, execute=True):
         #再设置当前姿态为起始姿态
        self.robot_arm.set_start_state_to_current_state()  
        #
        waypoints = []
        # wpose=self.robot_arm.get_current_pose().pose
        # wpose.position.x=  self.grasp_pose_wrist3.position.x
        # wpose.position.y=  self.grasp_pose_wrist3.position.y
        # wpose.position.z=  self.grasp_pose_wrist3.position.z
        waypoints.append(copy.deepcopy(pose))

        #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
        #笛卡尔路径运动 路点列表 终端布进值 最小移动值 避撞规划
        (plan, fraction) = self.robot_arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,         # jump_threshold
            True)        # avoid_collisions 避撞规划     
            ##显示轨迹
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_arm.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        #执行,并等待这个轨迹执行成功
        new_plan=self.scale_trajectory_speed(plan,0.3) #设定机械臂运动速度 0.3
        self.robot_arm.execute(new_plan,wait=True)
        # self.robot_arm.execute(plan,wait=True)

        #执行抓取
        # rospy.loginfo("Start to grasp")
        #self.set_gripper(0.01,epsilon=0.4)#张开3cm
        rospy.sleep(0.5)

    
    def scale_trajectory_speed(self,traj,spd=0.1):
        new_traj = RobotTrajectory()
        new_traj = traj

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)
        # print("n_joints:", n_joints)
        # print("n_points:", n_points)
        #spd = 3.0

        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions

            # print(point)
            for j in range(n_joints):
                # print("j:", j)
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd

            points[i] = point

        new_traj.joint_trajectory.points = points     
        return   new_traj


    # def get_parameters(self):
    #     pass
