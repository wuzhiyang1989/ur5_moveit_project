#!/home/ts/anaconda3/envs/YOLO/bin/python3

import rospy
import numpy
from detection.detection import *
from geometry_msgs.msg import Pose
from moveit_control.moveit_control import *
from ts_hand.ts_control import *

threshold = 0.2                                # 置信度阈值，调整大小适应新的模型 0.5
rospy.init_node('go_grasp', anonymous=True)
rospy.loginfo("Robot  initializing")

ur5 = URMoveITControl()
ts  = InteractiveInterface()
# dh  = ts_control()
ts.gripper_client(80, 1000)
# 设置机械臂加速度为 0.1
ur5.robot_arm.set_max_acceleration_scaling_factor(1)
ur5.robot_arm.set_max_velocity_scaling_factor(1)

while True:
    # 获取桌面图像
    image_flag = ts.image_client()

    # 保存图像的名称固定
    image_name = "/home/ts/Pictures/ts_ws/detect_crop.png"
    #print(image_name)
    resp = ts.object_client(image_name)
    print(resp)
    if not resp.detect_flag:
        continue
    print(resp.detect_flag)
    # 进入迭代器，循环抓取物品
    for object in resp.obj:
        if object.label == "bottle_spring":  # bottle
            if object.conf < threshold:                                 # object.conf 表示物品识别的置信度，大于 0.7 表示识别结果可信，可按照实际需求及表现进行修改
                print(f'发现目标物体：{object.label}, 由于置信度太低（{object.conf} < {threshold})而抛弃。')
            else:
                 # 移动机械臂准备抓取
                ur5.move_to_joints(ur5.robot_arm, vert_joint,tag="working")
                # 旋转机械臂末端适应位置
                #rad = (90 - object.angle) * numpy.pi / 180
                #ur5.move_one_joint(ur5.robot_arm, rad, 6)
                # 获取点云信息
                while True:
                    point = ts.get_point_client(object.colu-320+160, object.row-257+275)      # (160  275)桌面裁剪左上角坐标点
                    
                    if(point.position.x == None):
                        continue
                    else:
                        break
                # 坐标转换
                #point_t = ts.translate(point.x, point.y, point.z)
                point_t = ts.translate_service_client(point.position.x, point.position.y, point.position.z)
                
                # 移动机械臂到物体上方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = point_t.translated_position.x
                wpose_1.position.y = point_t.translated_position.y
                wpose_1.position.z = 0.4
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))

                '''
                point = ts.get_point_client(gripper.colu-320+160, gripper.row-257+275)
                point = ts.get_point_client(object.colu-320+160, object.row-257+275)

                '''
                # 下降机械臂
                wpose_2 = ur5.robot_arm.get_current_pose().pose
                wpose_2.position.z = 0.20                                           # 调节 Z 值为适合抓取的高度
                ur5.move_to_cartesian(copy.deepcopy(wpose_2))
                # 闭合夹爪
                # ts.gripper_client(80, 300)                    # 80->p：位置  300：距离
                #writebuf = bytearray(dh_projectstart_buf)
                # dh.ser_.flushInput()
                # dh.ser_.flushOutput()
                # dh.ts_write(writebuf)                           # 闭合夹爪
                # dh.ts_read()
                
                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.3)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.3)
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
                # writebuf = bytearray(dh_prjectend_buf)
                # dh.ser_.flushInput()
                # dh.ser_.flushOutput()
                # dh.ts_write(writebuf)                           # 打开夹爪
                # dh.ts_read()
                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.3)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.3)
                # 回到初始状态
                ur5.move_to_joints(ur5.robot_arm, init_joint,tag="working")
#             pass
        if object.label == "upright bottle": 
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
                wpose_1.position.x = point_t.translated_position.x                   # 机械臂右方
                wpose_1.position.y = 0.30                                            # 机械臂正前方
                wpose_1.position.z = 0.10
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 前进机械臂
                wpose_2 = ur5.robot_arm.get_current_pose().pose
                wpose_2.position.y = point_t.translated_position.y-0.16              # 0.16 二指爪偏移量
                ur5.move_to_cartesian(copy.deepcopy(wpose_2))
                # 夹爪闭合
                # ts.gripper_client(80, 300)
                writebuf = bytearray(dh_projectstart_buf)

                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.1)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.1)
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
                wpose_1.position.x = point_t.translated_position.x                   # 机械臂右方
                wpose_1.position.y = point_t.translated_position.y                   # 机械臂正前方
                wpose_1.position.z = 0.3
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 释放夹爪
                # ts.gripper_client(80, 990)
                writebuf = bytearray(dh_prjectend_buf)

                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(1)
                ur5.robot_arm.set_max_velocity_scaling_factor(1)
                # 回到初始状态
                ur5.move_to_joints(ur5.robot_arm, init_joint,tag="working")
    #             pass

#     rospy.loginfo("finished!")
    break

# rospy.spin()
