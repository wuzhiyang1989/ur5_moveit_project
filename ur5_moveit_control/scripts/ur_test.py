#!/home/ts/anaconda3/envs/YOLO/bin/python3
"""
import rospy
import numpy
from detection.detection import *
from geometry_msgs.msg import Pose
from moveit_control.moveit_control import *
from ts_hand.ts_control import *



threshold = 0.2                                # 置信度阈值，调整大小适应新的模型
"""

# from ur_kinematics import *
#from select_ik_solution import select_ik_solution
from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
# from Inverse_Kinematics import IK
 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

"""
 def ur5_init():
    rospy.init_node('go_grasp', anonymous=True)
    rospy.loginfo("Robot  initializing")

    ur5 = URMoveITControl()
    ts  = InteractiveInterface()

    # 设置机械臂加速度为 0.1
    ur5.robot_arm.set_max_acceleration_scaling_factor(1)
    ur5.robot_arm.set_max_velocity_scaling_factor(1)


def run_ur5_test():
    ur5_init()

    while True:
        input_fun = input("请输入: ")
        if input_fun == '1':
            break

# run_ur5_test()
"""
import numpy as np
from math import sin, cos, pi

#import Commonly_uesd_function as cup
from math import pi
#import ur_kinematics.ur_config

#a = ur_config.UR5_DH_param.a
#alpha = ur_config.UR5_DH_param.alpha
#d = ur_config.UR5_DH_param.d

def rotx(angle):
          T = np.matrix([[1,0,0,0],[0,cos(angle),-1*sin(angle),0],[0,sin(angle),cos(angle),0],[0,0,0,1]])
          return T

def roty(angle):
          T = np.matrix([[cos(angle),0,sin(angle),0],[0,1,0,0],[-1*sin(angle),0,cos(angle),0],[0,0,0,1]])
          return T

def rotz(angle):
          T = np.matrix([[cos(angle),-1*sin(angle),0,0],[sin(angle),cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
          return T

def trans(x,y,z):
          T=np.matrix([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
          return T



#选取逆运动学求解的8组结果中和position最接近的值
def select_ik_solution(position, solution):
    #weight各关节角的权重因子
    weight = [1]*6
    #kpi:solution中各解的得分，得分越低越好
    kpi = [0]*len(solution)
    for i in range(len(solution)):
            kpi[i] = weight[0]*abs(solution[i][0]-position[0])+weight[1]*abs(solution[i][1]-position[1])+weight[2]*abs(solution[i][2]-position[2])+weight[3]*abs(solution[i][3]-position[3])+weight[4]*abs(solution[i][4]-position[4])+weight[5]*abs(solution[i][5]-position[5])
    for i in range(len(kpi)):
            if kpi[i] == min(kpi):
                        break

    return solution[i]

def IK(T):
    nx = T[0][0]
    ny = T[1][0]
    nz = T[2][0]
    ox = T[0][1]
    oy = T[1][1]
    oz = T[2][1]
    ax = T[0][2]
    ay = T[1][2]
    az = T[2][2]
    px = T[0][3]
    py = T[1][3]
    pz = T[2][3]

    #求解theta1
    m=d[5]*ay-py
    n=ax*d[5]-px
    theta1 = [0]*8
    for i in range(4):
            theta1[i] = atan2(m,n)-atan2(d[3],sqrt(m**2+n**2-d[3]**2))
            theta1[i+4] = atan2(m,n)-atan2(d[3],-1*sqrt(m**2+n**2-d[3]**2))

    #求解theta5
    theta5 = [0]*8
    theta5[0] = acos(ax*sin(theta1[0])-ay*cos(theta1[0]))
    theta5[1] = theta5[0]
    theta5[2] = -1*theta5[0]
    theta5[3] = theta5[2]
    theta5[4] = acos(ax*sin(theta1[4])-ay*cos(theta1[4]))
    theta5[5] = theta5[4]
    theta5[6] = -1*theta5[4]
    theta5[7] = theta5[6]

    #求解theta6
    theta6 = [0]*8
    for i in range(8):
            m = nx*sin(theta1[i])-ny*cos(theta1[i])
            n = ox*sin(theta1[i])-oy*cos(theta1[i])
            theta6[i] = atan2(m,n)-atan2(sin(theta5[i]),0)
    
    #求解theta3
    theta3 = [0]*8
    m = [0]*8
    n = [0]*8
    for i in range(8):
            m[i] = d[4]*(sin(theta6[i])*(nx*cos(theta1[i])+ny*sin(theta1[i]))+cos(theta6[i])*(ox*cos(theta1[i])+oy*sin(theta1[i])))-d[5]*(ax*cos(theta1[i])+ay*sin(theta1[i]))+px*cos(theta1[i])+py*sin(theta1[i])
            n[i] = pz-d[0]-az*d[5]+d[4]*(oz*cos(theta6[i])+nz*sin(theta6[i]))
            if (i%2) == 0:
                        theta3[i] = acos((m[i]**2+n[i]**2-a[1]**2-a[2]**2)/(2*a[1]*a[2]))
            else:
                        theta3[i] = -1*acos((m[i]**2+n[i]**2-a[1]**2-a[2]**2)/(2*a[1]*a[2]))

    #求解theta2    
    theta2 = [0]*8
    s2 = [0]*8
    c2 = [0]*8
    for i in range(8):
            s2[i] = ((a[2]*cos(theta3[i])+a[1])*n[i]-a[2]*sin(theta3[i])*m[i])/(a[1]**2+a[2]**2+2*a[1]*a[2]*cos(theta3[i]))
            c2[i] = (m[i]+a[2]*sin(theta3[i])*s2[i])/(a[2]*cos(theta3[i])+a[1])
            theta2[i] = atan2(s2[i],c2[i])

    #求解theta4
    theta4 = [0]*8
    for i in range(8):
            theta4[i] = atan2(-1*sin(theta6[i])*(nx*cos(theta1[i])+ny*sin(theta1[i]))-cos(theta6[i])*(ox*cos(theta1[i])+oy*sin(theta1[i])),oz*cos(theta6[i])+nz*sin(theta6[i]))-theta2[i]-theta3[i]        

    solution = [0]*8
    for i in range(8):
            solution[i] = control_angles([theta1[i],theta2[i],theta3[i],theta4[i],theta5[i],theta6[i]])

    return solution

def move():
    #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
    goal = FollowJointTrajectoryGoal()

    #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
    goal.trajectory = JointTrajectory()
    
    goal.trajectory.joint_names = JOINT_NAMES

    #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时要第一个值为参考
    joint_states = rospy.wait_for_message("joint_states",JointState)
    joints_pos = joint_states.position

    #tips：
    joints_pos_list = list(joints_pos)
    record = joints_pos_list[2]
    joints_pos_list[2] = joints_pos_list[0]
    joints_pos_list[0] = record

    #设定目标姿态T，该T是事先通过正运动学模块进行求解，并通过逆运动学进行验证求解，确保逆运动学有解
    x= -722.6
    T = [[-1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, -109.15000000000002], [0.0, 0.0, -1.0, 7.159000000000006], [0.0, 0.0, 0.0, 1.0]]
    joint_position = [0]*20
    solution = IK(T)
    joint_position[0] = select_ik_solution(joints_pos_list, solution)

    for i in range(1,20):
            #在笛卡尔空间下设置20个点，每个点只在x方向移动10，其余方向维持不变
            x = x+10
            T = [[-1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, -109.15000000000002], [0.0, 0.0, -1.0, 7.159000000000006], [0.0, 0.0, 0.0, 1.0]]
            #逆运动学求解每一个点的关节角度值，每一次求解均有8组解
            solution = IK(T)
            #选出与上一次求解结果最接近的解
            joint_position[i] = select_ik_solution(joint_position[i-1], solution)
    
    time = 0.3
    goal.trajectory.points=[0]*20
    goal.trajectory.points[0]=JointTrajectoryPoint(positions=joints_pos,time_from_start=rospy.Duration(0))
    for i in range(1,20):
            #将所有解赋给goal.trajectory.points
            goal.trajectory.points[i]=JointTrajectoryPoint(positions=joint_position[i],time_from_start=rospy.Duration(time))
            time = time+0.3
    client.send_goal(goal)
 
def pub_test():
    global client

    #初始化ros节点
    # rospy.init_node("pub_action_test")

    #实例化一个action的类，命名为client，与上述client对应，话题为arm_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
    # client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server...")
    #等待server
    # client.wait_for_server()
    print("Connect to server")
    #执行move函数，发布action
    move()
 
if __name__ == "__main__":
    pub_test()


"""
while True:
    # 进入迭代器，循环抓取物品
    for object in resp.obj:
        if object.label == "bottle":
            if object.conf < threshold:                                 # object.conf 表示物品识别的置信度，大于 0.7 表示识别结果可信，可按照实际需求及表现进行修改
                print(f'发现目标物体：{object.label}, 由于置信度太低（{object.conf} < {threshold})而抛弃。')
            else:
                 # 移动机械臂准备抓取
                ur5.move_to_joints(ur5.robot_arm, vert_joint,tag="working")
                # 旋转机械臂末端适应位置
                rad = (90 - object.angle) * numpy.pi / 180
                ur5.move_one_joint(ur5.robot_arm, rad, 6)
                # 获取点云信息
                while True:
                    point = ts.get_point_client(object.colu-320+160, object.row-257+275)      # (160  275)桌面裁剪左上角坐标点
                    
                    if(point.position.x == None):
                        continue
                    else:
                        break
                # 坐标转换
                # point_t = ts.translate(point.x, point.y, point.z)
                point_t = ts.translate_service_client(point.position.x, point.position.y, point.position.z)
                
                # 移动机械臂到物体上方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = point_t.translated_position.x
                wpose_1.position.y = point_t.translated_position.y
                wpose_1.position.z = 0.4
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 下降机械臂
                wpose_2 = ur5.robot_arm.get_current_pose().pose
                wpose_2.position.z = 0.20                                           # 调节 Z 值为适合抓取的高度
                ur5.move_to_cartesian(copy.deepcopy(wpose_2))
                # 闭合夹爪
                # ts.gripper_client(80, 300)                    # 80->p：位置  300：距离
                writebuf = bytearray(dh_projectstart_buf)
                dh.ser_.flushInput()
                dh.ser_.flushOutput()
                dh.ts_write(writebuf)                           # 闭合夹爪
                dh.ts_read()
                
                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.1)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.1)
                # 机械臂回到竖直状态
                # ur5.move_to_joints(ur5.robot_arm, vert_joint,tag="working")
                # 准备放置
                resp = ts.target_client(image_name)
                print(resp)
                # # 获取点云信息
                point = ts.get_point_client(resp.obj.colu+160, resp.obj.row+275)     # （160   275）同上
                # 坐标转换
                point_t = ts.translate_service_client(point.position.x, point.position.y, point.position.z)
                # 移动机械臂到物体上方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = point_t.translated_position.x                   # 机械臂右方
                wpose_1.position.y = point_t.translated_position.y                   # 机械臂正前方
                wpose_1.position.z = 0.3                                             # 调整机械臂末端Z值
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 释放夹爪
                # ts.gripper_client(80, 990)
                writebuf = bytearray(dh_prjectend_buf)
                dh.ser_.flushInput()
                dh.ser_.flushOutput()
                dh.ts_write(writebuf)                           # 打开夹爪
                dh.ts_read()
                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(1)
                ur5.robot_arm.set_max_velocity_scaling_factor(1)
                # 回到初始状态
                ur5.move_to_joints(ur5.robot_arm, init_joint,tag="working")
#             pass
        #if object.label == "upright bottle": 
        if object.label == "bottle_spring": 
            if object.conf < threshold:                                 # object.conf 表示物品识别的置信度，大于 0.7 表示识别结果可信，可按照实际需求及表现进行修改
                print(f'发现目标物体：{object.label}, 由于置信度太低（{object.conf} < {threshold})而抛弃。')
            else:
                # 移动机械臂准备抓取
                ur5.move_to_joints(ur5.robot_arm, hori_joint,tag="working")
                # 获取点云信息
                while True:
                    point = ts.get_point_client(object.colu-320+160, object.row-257+275)
                    print("point:",point.position.x-0.1)
                    print("type = ", type(point.position.x))
                    if point.position.x is None:
                        continue
                    else:
                        break
                # 坐标转换
                # point_t = ts.translate(point.x, point.y, point.z)
                point_t = ts.translate_service_client(point.position.x, point.position.y, point.position.z)
                print("point_t:", point_t)
                
                # 移动机械臂到物体前方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = point_t.translated_position.x+0.122                # 机械臂右方 0.072
                wpose_1.position.y = point_t.translated_position.y-0.188            # 机械臂正前方 -0.188 
                wpose_1.position.z = 0.25
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 前进机械臂
                #wpose_2 = ur5.robot_arm.get_current_pose().pose
                #wpose_2.position.y = point_t.translated_position.y-0.16              # 0.16 二指爪偏移量
                #ur5.move_to_cartesian(copy.deepcopy(wpose_2))

                wpose_21 = ur5.robot_arm.get_current_pose().pose
                wpose_21.position.z = 0.04              # 0.16 二指爪偏移量
                ur5.move_to_cartesian(copy.deepcopy(wpose_21))

                # 夹爪闭合
                # ts.gripper_client(80, 300)
                writebuf = bytearray(dh_projectstart_buf)
                dh.ser_.flushInput()
                dh.ser_.flushOutput()
                dh.ts_write(writebuf)                           # 打开夹爪
                dh.ts_read()
                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.1)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.1)

                wpose_3 = ur5.robot_arm.get_current_pose().pose
                wpose_3.position.z = 0.2              # 0.16 二指爪偏移量
                ur5.move_to_cartesian(copy.deepcopy(wpose_3))

                # 机械臂回到竖直状态
                #ur5.move_to_joints(ur5.robot_arm, vert_joint, tag="working")
                # 准备放置
                resp = ts.target_client(image_name)
                print(resp)
                # # 获取点云信息
                point = ts.get_point_client(resp.obj.colu+160, resp.obj.row+275)     # （160 275）同上
                # 坐标转换
                point_t = ts.translate_service_client(point.position.x, point.position.y, point.position.z)
                # 移动机械臂到物体上方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = point_t.translated_position.x+0.06                 # 机械臂右方
                wpose_1.position.y = point_t.translated_position.y-0.23                   # 机械臂正前方0.24  
                wpose_1.position.z = 0.1
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))

                wpose_3 = ur5.robot_arm.get_current_pose().pose
                wpose_3.position.z = 0.047             # 0.16 二指爪偏移量
                ur5.move_to_cartesian(copy.deepcopy(wpose_3))
                # 释放夹爪
                # ts.gripper_client(80, 990)
                writebuf = bytearray(dh_prjectend_buf)
                dh.ser_.flushInput()
                dh.ser_.flushOutput()
                dh.ts_write(writebuf)                           # 打开夹爪
                dh.ts_read()
                # 设置机械臂加速度为 0.1

                ur5.robot_arm.set_max_acceleration_scaling_factor(0.5)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.5)
                wpose_32 = ur5.robot_arm.get_current_pose().pose
                wpose_32.position.z = 0.3             # 0.16 二指爪偏移量
                ur5.move_to_cartesian(copy.deepcopy(wpose_32))

                ur5.robot_arm.set_max_acceleration_scaling_factor(0.5)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.5)
                # 回到初始状态
                ur5.move_to_joints(ur5.robot_arm, init_joint,tag="working")
    #             pass

#     rospy.loginfo("finished!")
    break

# rospy.spin()
"""