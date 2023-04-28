#!/home/wy/anaconda3/envs/daily/bin/python3

import numpy as np
import pandas as pd
# from widget import Widget
import time
# from demo_orderpicking import *
import rospy
import threading
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from hm_cooperation.msg import Observe
import json
import xlrd


# data processing node
class DataProcess():
    def __init__(self) -> None:
        # define order picking task PARAMS
        self.ORDER = pd.read_excel("/home/wy/t_ws/src/hm_cooperation/scripts/data/test_order_1.xlsx").values
        f = open("/home/wy/t_ws/src/hm_cooperation/scripts/data/env_params.json",encoding='utf-8')        # 打开任务环境参数文件
        data = json.load(f)                         # 加载观测数据至 data
        
        # Env params
        self.N_BIN = data['N_BIN']                                  # product 数量
        self.N_SCANNER = data['N_SCANNER']                          # scanner 数量
        self.N_ORDER = data['N_ORDER']                              # order 数量

        self.N_POSITION = self.N_BIN+self.N_SCANNER+self.N_ORDER    # 合法位置数量
        self.BIN_INDEX_FROM = 0                                     # bin 位置起始
        self.BIN_INDEX_TO = self.N_BIN                              # bin 位置终止
        self.SCANNER_INDEX_FROM = self.N_BIN                        # scanner 位置起始
        self.SCANNER_INDEX_TO = self.N_BIN + self.N_SCANNER         # scanner 位置终止
        self.ORDER_INDEX_FROM = self.N_BIN + self.N_SCANNER         # order 位置起始
        self.ORDER_INDEX_TO = self.N_POSITION                       # order 位置终止
        self.ROBOT_INDEX = self.N_POSITION                          # Robot 位置
        self.HUMAN_INDEX = self.N_POSITION + 1                      # Human 位置
        self.RECYCLE_INDEX = self.N_POSITION + 2                    # 回收位置
        self.NALL_POSITION = self.N_POSITION + 3
        
        # Task params
        self.timeinfo = np.array(data['TIME_INFO'])                           # 任务时间参数
        self.cargoinfo = data['CARGO_INFO']                         # product 是否可被 Robot 执行
        
        # product position
        self.product_position_new = np.zeros((self.N_BIN,self.N_POSITION+3),dtype=int)
        for i in range(self.N_BIN):
            self.product_position_new[i,i] = 1    
        self.product_position_old = self.product_position_new+0

        # if product has been detected
        self.product_detected = np.zeros(self.N_BIN,dtype=int)
        
        # robot_action
        self.robot_action_new = np.array([self.N_POSITION,self.N_POSITION,-1],dtype=int)
        self.robot_action_old = self.robot_action_new+0
        
        # pod state
        self.pod_state_new = np.ones(self.N_BIN,dtype=int)
        self.pod_state_old = self.pod_state_new+0
        
        # scanner,order state
        self.scanner_order_state_new = np.zeros((self.N_SCANNER+self.N_ORDER+1,self.N_BIN),dtype=int)
        self.scanner_order_state_old = self.scanner_order_state_new+0
        
        # receive topics
        rospy.init_node('data_process', anonymous=True)
        # camera 1
        rospy.Subscriber("/transfer_state", Int8MultiArray, self.update_scanner_order_state)
        # camera 2
        rospy.Subscriber("/goods_shelves", Int8MultiArray, self.update_pod_state)
        # robot action
        rospy.Subscriber("/franka_task", Int8MultiArray, self.update_robotaction)        ##TODO: 通过 topic 接收 robot 正在执行的任务
        self.add_thread = threading.Thread(target=self.spin_thread)
        self.add_thread.start()

        # send topic
        # send robot command topic                                                          ##TODO: 通过 topic 发送观测{O,C,P,A} 至 planner节点
        self.observation_pub = rospy.Publisher('/Observation', Observe, queue_size=1)
        self.observation_msg = Observe()
        # 观测值 action 第一个维度
        action_dim_1 = MultiArrayDimension()
        action_dim_1.size   = 2             # 第一个维度的长度
        action_dim_1.stride = 2             # 整个矩阵大小
        action_dim_1.label  = ""            # 第一个维度标签
        # 观测值 action 第二个维度
        action_dim_2 = MultiArrayDimension()
        action_dim_2.size   = 2             # 第二个维度长度
        action_dim_2.stride = 2             # 第二个维度大小
        action_dim_2.label  = ""            # 第二个维度标签
        self.observation_msg.action.layout.dim = [action_dim_1, action_dim_2]

        # 观测值 order 第一个维度
        order_dim_1 = MultiArrayDimension()
        order_dim_1.size   = 2             # 第一个维度的长度
        order_dim_1.stride = 2             # 整个矩阵大小
        order_dim_1.label  = ""            # 第一个维度标签
        # 观测值 order 第二个维度
        order_dim_2 = MultiArrayDimension()
        order_dim_2.size   = 2             # 第二个维度长度
        order_dim_2.stride = 2             # 第二个维度大小
        order_dim_2.label  = ""            # 第二个维度标签
        self.observation_msg.position.layout.dim = [order_dim_1, order_dim_2]

        # 观测值 position 第一个维度
        position_dim_1 = MultiArrayDimension()
        position_dim_1.size   = 2             # 第一个维度的长度
        position_dim_1.stride = 2             # 整个矩阵大小
        position_dim_1.label  = ""            # 第一个维度标签
        # 观测值 position 第二个维度
        position_dim_2 = MultiArrayDimension()
        position_dim_2.size   = 2             # 第二个维度长度
        position_dim_2.stride = 2             # 第二个维度大小
        position_dim_2.label  = ""            # 第二个维度标签
        self.observation_msg.position.layout.dim = [position_dim_1, position_dim_2]

        # send interface topic
        self.interface_pub = rospy.Publisher('interface', Int8MultiArray, queue_size=1)
        self.interface_msg = Int8MultiArray()
        interface_dim = MultiArrayDimension()
        interface_dim.size   = 18
        interface_dim.stride = 18
        interface_dim.label  = "interface cmd"
        self.interface_msg.layout.dim.append(interface_dim)
        pass
    
    # 根据 topic 更新 robot action
    def update_robotaction(self,robot_a):
        robot_a = np.array(robot_a.data,dtype=int).squeeze()
        # 如果检测到新的值则更新
        if np.any(robot_a != self.robot_action_new):
            self.robot_action_old = self.robot_action_new+0
            self.robot_action_new = robot_a
            self.update_product_position()
        pass
    
    # 根据 topic 更新 pod state
    def update_pod_state(self,pod_state):
        pod_state = np.array(pod_state.data,dtype=int).squeeze()
        # 如果检测到新的值则更新
        if np.any(self.pod_state_new!=pod_state):
            self.pod_state_old = self.pod_state_new+0
            self.pod_state_new = pod_state
            self.update_product_position()
        pass
    
    # 根据 topic 更新 scanner, order 
    def update_scanner_order_state(self,scanner_order_state):
        
        scanner_order_state = np.array(scanner_order_state.data,dtype=int).reshape(self.N_ORDER+self.N_SCANNER+1,self.N_BIN).transpose()
        # 如果检测到新的值则更新
        if np.any(self.scanner_order_state_new!=scanner_order_state):
            self.scanner_order_state_old = self.scanner_order_state_new+0
            self.scanner_order_state_new = scanner_order_state
            self.update_product_position()
        pass
    
    # 根据所有 topic 内容更新 product position
    def update_product_position(self):
        self.product_position_old = self.product_position_new+0     # 记录 old position
        self.product_position_new = np.zeros((self.N_BIN,self.N_POSITION+3),dtype=int)    # 全部初始化为0
        for i in np.nonzero(self.pod_state_new)[0]:     # pod 中的 product 置为1
            self.product_position_new[i,i] = 1
        index = np.nonzero(self.scanner_order_state_new)
        for i in range(len(index[0])):   # scanner, order 中的 product 置为1
            j = index[1][i]+self.N_BIN
            if j>=self.N_POSITION:
                self.product_position_new[index[0][i],self.RECYCLE_INDEX] = 1    
            self.product_position_new[index[0][i],j] = 1

        p_in_robot = self.robot_action_new[-1]
        if p_in_robot>=0 and p_in_robot<self.N_POSITION:
            self.product_position_new[p_in_robot,self.ROBOT_INDEX] = 1
            
        pass
    
    
    # 发送 joint observation
    def send_joint_observation(self):

        # human action 初值为无效值
        action_h = np.array([self.NALL_POSITION,self.NALL_POSITION,self.N_BIN],dtype=int)
        product_position = self.product_position_new[:,self.BIN_INDEX_FROM:self.ORDER_INDEX_TO]
        unseen_product_idx = list(np.where(np.sum(product_position,axis=1)==0)[0])          # 不在 POD、SCANNER、ORDER 内的 product
        product_in_h = []
        for p in unseen_product_idx:
            if p != self.robot_action_new[-1]:                      # 如果 product 同时不在 R 中，则认为在 H 中
                product_in_h.append(p)
        for p in product_in_h:                                    # 如果认为在 H 中的 product 有效
            action_h[-1] = p                               # 设置 action_h
            self.product_position_new[p,self.HUMAN_INDEX] = 1    # 设置 product position 为 H 
            
        if action_h[-1]<self.N_BIN:     # 如果 human action 有效
            product_ = action_h[-1]     # product_ 是 human action 的 product id
            if np.count_nonzero(self.product_position_old[product_,:])>0:           # 如果在 old position 里存在 product
                index = np.nonzero(self.product_position_old[product_,:])[0][0]         # 根据 old position 决定 action_h 其它量
                # 如果 H 所拿物品之前的位置在 POD
                if index < self.BIN_INDEX_TO:
                    action_h[0] = product_
                # 否则 H 所拿的物品之前的位置在 SCANNER
                else:
                    action_h[0] = index
                    # 如果 H 所拿物品是订单中所需物品，设置目标位置为响应订单位置
                    if np.count_nonzero(self.ORDER[product_,:])>0:
                        order_index = np.nonzero(self.ORDER[product_,:])[0][0]
                        action_h[1] = order_index
                    # 否则设置目标位置为 RECYCLE 位置
                    else:
                        action_h[1] = self.RECYCLE_INDEX

        # 设置 action_r 导致的 product_position 更新
        action_r = self.robot_action_new
        # if action_r[-1]>0 and action_r[-1]<self.N_BIN:
        #     if np.sum(self.product_position_new[action_r[-1],:])==0:
        #         self.product_position_new[action_r[-1],self.ROBOT_INDEX]=1

        action = np.concatenate((self.robot_action_new,action_h))
        actionlist = action.tolist()

        print(f'send_joint_observation: sending position from camera:\n{self.product_position_new}')
        print(f'send_joint_observation: sending ongoing task: {action}')
        
        # 给观测值赋值
        self.observation_msg.timebound = 80
        self.observation_msg.action.data = actionlist
        self.observation_msg.order.data = self.ORDER.reshape(3*12).tolist()
        self.observation_msg.position.data = self.product_position_new.reshape(12*24).tolist()

        # 发布观测值
        # if all(np.sum(self.product_position_new,axis=1)==1):
        self.observation_pub.publish(self.observation_msg)
        ##! disp self.product_position_new
        # print(f'online O is: \n{O}\n')
        # print(f'online C is: \n{C}\n')
        # print(f'online P is: \n{P}\n')
        # print(f'data_process.product_position_new is\n{self.product_position_new}\n')
        # print(f'detected is \n {self.product_detected}\n')
        # print(f'online action is: \n{action}')
        # print('='*80)
        
        ##! send action
        
        pass    
    
    def send_scanning_results(self):
        
        order_state = self.product_position_new[:,self.ORDER_INDEX_FROM:self.ORDER_INDEX_TO]
        order_state = self.ORDER - order_state
        
        ##! products for highlight
        product_for_pick = np.zeros(self.N_BIN, dtype=int)        
        for i in range(self.N_BIN):
            index = np.nonzero(order_state[i,:])[0]
            if index.size>0 and self.pod_state_new[i]>0:
                product_for_pick[i] = 1
        
        
        ##! scanning results for highlight
        scanning_result = np.zeros(self.N_SCANNER, dtype=int)     # init all 0
        # products positions in scanner 
        scanner_state = self.product_position_new[:,self.SCANNER_INDEX_FROM:self.SCANNER_INDEX_TO]
        occ_index = np.nonzero(scanner_state)    # product in non-empty scanner
        # if product in scanner is in order, pick it  
        for i in range(occ_index[0].size):
            product_ = occ_index[0][i]
            scanner_ = occ_index[1][i]
            order_index = np.nonzero(order_state[product_,:])[0]
            if order_index.size>0:
                scanning_result[scanner_] = order_index[0] + 1
            else:
                scanning_result[scanner_] = self.N_ORDER + 1
        print(f'product_for_pick is {product_for_pick}\n')
        print(f'scanning_result is {scanning_result}\n')
        data = np.concatenate((product_for_pick,scanning_result.reshape(1,-1).squeeze())).tolist()
        
        self.interface_msg.data = data
        # print("="*80)
        # print("data = ", data)
        self.interface_pub.publish(self.interface_msg)

    def spin_thread(self):
        rospy.spin()


def main():
    
    data_process = DataProcess()    # 初始化 data process 节点
    # rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        # data_process.update_product_position()

        # # 发送 joint observation
        data_process.send_joint_observation()

        # data_process.update_product_position()

        # # 发送 scanning results
        data_process.send_scanning_results()
        # print(f'data_process.product_position_new is {data_process.product_position_new}\n')
        # print(f'data_process.product_position_old is {data_process.product_position_old}\n')
        # print('='*80)
        # data_process.send_joint_observation()
        rate.sleep()

if __name__ == "__main__":
    main()