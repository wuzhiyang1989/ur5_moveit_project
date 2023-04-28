#!/home/ts/anaconda3/envs/YOLO/bin/python3

import time
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
    # 保存图像的名称 
    # 修改图片名称
    image_name = "detect.png"
    resp = ts.object_client(image_name)
    # print(resp)
    if not resp.detect_flag:
        continue
    # print(resp.detect_flag)
    # 获取容器位置
    # container = None
    # if resp.obj.count('box') == 0:
    #     continue                       # 未检测到容器，重新检测
    # for box in resp.obj:
    #     if box.label == 'box':         # 检测到容器
    #         if container == None:
    #             container = box
    #         else:
    #             if container.conf < box.conf:
    #                 container = box

    # container_point = ts.get_point_client(container.colu+250, container.row+300)
    
    # # 坐标转换
    # container_point_t = ts.translate_service_client(container_point.position.x, container_point.position.y, container_point.position.z)

    # 进入迭代器，循环抓取物品
    for object in resp.obj:
        #if object.label == "box":    # 返回值为 'box'，记得修改标签
            #continue
        if object.label == "bottle":  # bottle，躺倒物体抓取末模板
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
                    obj_point = ts.get_point_client(object.colu+250, object.row+300)
                    
                    if(obj_point.position.x == None):
                        continue
                    else:
                        break
                # 坐标转换
                #obj_point_t = ts.translate(obj_point.x, obj_point.y, obj_point.z)
                obj_point_t = ts.translate_service_client(obj_point.position.x, obj_point.position.y, obj_point.position.z)
                
                # 移动机械臂到物体上方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = obj_point_t.translated_position.x
                wpose_1.position.y = obj_point_t.translated_position.y
                wpose_1.position.z = 0.8
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                
                # 下降机械臂
                wpose_2 = ur5.robot_arm.get_current_pose().pose
                wpose_2.position.z = 0.6                                          # 调节 Z 值为适合抓取的高度
                ur5.move_to_cartesian(copy.deepcopy(wpose_2))
                exit(0)
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
                
                # 移动机械臂到容器上方
                # wpose_1 = ur5.robot_arm.get_current_pose().pose
                # wpose_1.position.x = container_point_t.translated_position.x                   # 机械臂右方
                # wpose_1.position.y = container_point_t.translated_position.y                   # 机械臂正前方
                # wpose_1.position.z = 0.3                                             # 调整机械臂末端Z值
                # ur5.move_to_cartesian(copy.deepcopy(wpose_1))
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
        if object.label == "bottle_spring":    # 站立物体抓取模板
            if object.conf < threshold:                                 # object.conf 表示物品识别的置信度，大于 0.7 表示识别结果可信，可按照实际需求及表现进行修改
                print(f'发现目标物体：{object.label}, 由于置信度太低（{object.conf} < {threshold})而抛弃。')
            else:
                # 移动机械臂准备抓取
                ur5.move_to_joints(ur5.robot_arm, hori_joint,tag="working")
                # 获取点云信息
                while True:
                    obj_point = ts.get_point_client(object.colu+250, object.row+300)
                    if obj_point.position.x is None:
                        continue
                    else:
                        break
                # 坐标转换
                # point_t = ts.translate(point.x, point.y, point.z)
                obj_point_t = ts.translate_service_client(obj_point.position.x, obj_point.position.y, obj_point.position.z)
                
                # 移动机械臂到物体前方
                wpose_1 = ur5.robot_arm.get_current_pose().pose
                wpose_1.position.x = obj_point_t.translated_position.x + 0.03             # 机械臂右方
                wpose_1.position.y = obj_point_t.translated_position.y - 0.2                                  # 机械臂正前方
                wpose_1.position.z = obj_point_t.translated_position.z + 0.2
                ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                
                # 前进机械臂
                wpose_2 = ur5.robot_arm.get_current_pose().pose
                #wpose_2.position.y = obj_point_t.translated_position.y              # 0.16 二指爪偏移量
                wpose_2.position.z = 0.15
                ur5.move_to_cartesian(copy.deepcopy(wpose_2))
                exit(0)
                # 夹爪闭合
                # ts.gripper_client(80, 300)
                # writebuf = bytearray(dh_projectstart_buf)

                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(0.1)
                ur5.robot_arm.set_max_velocity_scaling_factor(0.1)
                # 机械臂回到竖直状态
                #ur5.move_to_joints(ur5.robot_arm, vert_joint, tag="working")
                # 准备放置
                # 移动机械臂到容器上方
                # wpose_1 = ur5.robot_arm.get_current_pose().pose
                # wpose_1.position.x = container_point_t.translated_position.x                   # 机械臂右方
                # wpose_1.position.y = container_point_t.translated_position.y                   # 机械臂正前方
                # wpose_1.position.z = 0.3
                # ur5.move_to_cartesian(copy.deepcopy(wpose_1))
                # 释放夹爪
                # ts.gripper_client(80, 990)
                # writebuf = bytearray(dh_prjectend_buf)

                # 设置机械臂加速度为 0.1
                ur5.robot_arm.set_max_acceleration_scaling_factor(1)
                ur5.robot_arm.set_max_velocity_scaling_factor(1)
                # 回到初始状态
                ur5.move_to_joints(ur5.robot_arm, init_joint,tag="working")
    #             pass

#     rospy.loginfo("finished!")
    break

# rospy.spin()
