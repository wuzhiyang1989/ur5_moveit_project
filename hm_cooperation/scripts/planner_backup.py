#!/home/wy/anaconda3/envs/daily/bin/python3

import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import pandas as pd
import json
from utilis import *
import time

import threading
import rospy
from hm_cooperation.msg import Observe
from hm_cooperation.srv import *
from std_msgs.msg import Int8MultiArray, MultiArrayDimension



##===========================================================================================================
## CLASS TASKASSIGNMENT
##  member
##      - taskEnv <type SortingEnv>
##      - tasksetTotal <type SortingTaskSet>
##      - tasksetRobot <type SortingTaskSet>
##      - tasksetHuman <type SortingTaskSet>
##      - tasksetHumanBound <type SortingTaskSet>
##      - assignment <type ndarray> shape (ntask,2), [robot weight, human weight]
##  method
##      - optimize_assign, optimize assignment weights
##      - optimize_assign_XS, optimize assignment weights and, XS.size in robot > 0
##      - optimize_assign_Y, optimize assignment weights and, Y.size in robot > 0
##      - get_human_task_bound, get low bound of tasks of HUMAN to guarantee expectation constraints
##      - schedule_task, pop information of task and communication of robot
##      - plot_assignment, draw assignment 
##
class TaskAssignment:
    
    def __init__(self,env:TaskEnv) -> None:
        
        # Planning 变量
        self.observation = Observation()        # 观测值，product_position
        self.tasksetTotal = TaskSet()           # pick-and-place 任务集合
        self.tasksetRobot = []                  # 分配至 Robot 任务子集
        self.tasksetHuman = []                  # 分配至 Human 任务子集
        self.tasksetHumanBound = []             # Human 最小任务子集
        self.assignment = []                    # 任务分配权重
        self.action = []                        # action 解算结果
        self.timebound = env.TIME_BOUND         # 时间约束
        self.reassign_flag = False              # 重规划 flag
        self.env = env                          # 任务环境
        self.order = np.zeros((env.N_BIN,env.N_ORDER),dtype=int)
        for o_ in range(env.N_ORDER):
            for p in env.ORDER[o_]:
                self.order[p,o_]=1                         
        # Topics
        # receive topics
        rospy.init_node('data_process', anonymous=True)
        # observation from data_process
        rospy.Subscriber("/Observation", Observe, self.update_observation)
        self.add_thread = threading.Thread(target=self.spin_thread)
        self.add_thread.start()
        # send topics
        # command of robot next task 
        self.command_pub = rospy.Publisher('franka_cmd', Int8MultiArray, queue_size=1)
        self.command_msg = Int8MultiArray()
        command_dim = MultiArrayDimension()
        command_dim.size   = 3
        command_dim.stride = 3
        command_dim.label  = "cmd"
        self.command_msg.layout.dim.append(command_dim)
        # command of communication
        self.comm_pub = rospy.Publisher('comm', Int8MultiArray, queue_size=1)
        self.comm_msg = Int8MultiArray()
        comm_dim = MultiArrayDimension()
        comm_dim.size   = 18
        comm_dim.stride = 18
        comm_dim.label  = "cmd"
        self.comm_msg.layout.dim.append(comm_dim)
        
        pass  
        
        
    def actiontype(self,action):
        
        ## 仅接受 list(tuple) 类型的输入，且长度为3
        if type(action)!=list:
            print(f'Function actiontype only acceptes list(shape: 3,) ...')
            return 'Null'
        elif len(action)!=3:
            print(f'Function actiontype only acceptes list(shape: 3,) ...')
            return 'Null'
        
        ## 若 ongoing 的任务中的物品id非法，则返回空任务
        if not (action[-1]>=0 and action[-1]<self.env.N_BIN):
            return 'Null'
        ## 若 ongoing 任务起点为 POD，则任务类型为 POD2SCANNER
        elif action[0]>=self.env.BIN_INDEX_FROM and action[0]<self.env.BIN_INDEX_TO:
            return 'POD2SCANNER'
        ## 若 ongoing 任务终点为 ORDER，则任务类型为 SCANNER2ORDER
        elif action[1]>=self.env.ORDER_INDEX_FROM and action[1]<self.env.ORDER_INDEX_TO:
            return 'SCANNER2ORDER'
        ## 若 ongoing 任务终点为 RECYCLE，则任务类型为 SCANNER2RECYCLE
        elif action[1]==self.env.RECYCLE_INDEX:
            return 'SCANNER2RECYCLE'
        ## 在 ongoing 任务中的物品有效，但无有效起点和终点时，报错
        else:
            raise Exception(f'Not enough params are given for action type...')
        
    ## assign
    def assign(self,observation=None,timebound=[]):
        if timebound!=[]:
            self.timebound = timebound
            
        # time_begin = time.time()
        
        ## 函数输入 observation 用于仿真验证
        if observation is not None:
            self.update_observation(observation)

        ## 如果任务完成，返回空
        if self.done:
            return [self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN,self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN]
        
        ## 2. 不需 reassign 则返回之前的 action
        if not self.reassign_flag:
            
            # time_end = time.time()
            # self.timebound -= time_end-time_begin
            # self.timebound = np.maximum(self.timebound,0)
            return self.action
            
        ## 3. 重新优化分配
        self.optimize_assign()
        self.get_human_task_bound()           # Human 最小任务约束集合
        
        # 初始化 comm, 初始化 command
        command = PickPlaceTask()
        comm = PickPlaceTask()
        
        # 根据 observation 决定 command,comm
        # 如果分配到 R 的任务集合中有可执行的任务，则执行该任务
        # 否则执行可执行的任务
        robot_tasks_idx = self.task_available(self.tasksetRobot)
        if len(robot_tasks_idx)>0:
            task = self.tasksetRobot.tasks[robot_tasks_idx[0]]
            command = task
            print(f'chose from robot taskset')
        else:
            all_tasks_idx = self.task_available(self.tasksetTotal)
            for t in all_tasks_idx:
                if self.query_robot_do_task(self.tasksetTotal.tasks[t]):
                    command = self.tasksetTotal.tasks[t]
                    print(f'chose from totalset')
                    break
        
        ## 5. 根据 HumanBound 决定是否需要提示 Human
        assign_mode = 'OURS'
        # assign_mode = 'SR'
        ##assign_mode = 'SO'
        ##assign_mode = 'HC'
        ## OURS method
        if assign_mode == 'OURS':
            if comm.isempty():
                # 如果 necessary bound 集合内的任务数量过大，则增加 communication
                # 如果 bound 任务集合中的任务数量多于原有任务数量的50%
                # 且 bound 任务集合数量>2，则从 bound 中选择可执行任务作为 communication 内容
                if self.tasksetHumanBound.ntask>0.5*self.tasksetHuman.ntask and self.tasksetHumanBound.ntask>2:
                    # 查询是否有可执行的 SCANNER 2 RECYCLE 任务
                    t1 = self.pop_scanner2recycle(self.tasksetHumanBound)
                    if len(t1)>0:
                        comm = self.tasksetHumanBound.tasks[t1[0]]
                    else:                  
                        t2 = self.pop_pod(self.tasksetHumanBound)
                        if len(t2)>0:
                            comm = self.tasksetHumanBound.tasks[t2[0]]
                        else:
                            t3 = self.pop_scanner(self.tasksetHumanBound)
                            if len(t3)>0:
                                comm = self.tasksetHumanBound.tasks[t3[0]]
                            else:
                                t4 = self.task_available(self.tasksetTotal)
                                if len(t4)>0 and self.observation.action_h[-1]>=self.env.N_BIN:
                                    for t in t4:
                                        if command.isempty():
                                            comm = self.tasksetTotal.tasks[t]
                                        elif self.tasksetTotal.tasks[t].product!=command.product:
                                            comm = self.tasksetTotal.tasks[t]
                                            
                                            
        elif assign_mode == 'SR':
            all_tasks_idx = self.task_available(self.tasksetTotal)
            for t in all_tasks_idx:
                if self.query_robot_do_task(self.tasksetTotal.tasks[t]):
                    command = self.tasksetTotal.tasks[t]
            comm = PickPlaceTask()
            
            
        elif assign_mode == 'SO':
            t1 = self.pop_scanner2recycle(self.tasksetHumanBound)
            if len(t1)>0:
                comm = self.tasksetHumanBound.tasks[t1[0]]
            else:                  
                t2 = self.pop_pod(self.tasksetHumanBound)
                if len(t2)>0:
                    comm = self.tasksetHumanBound.tasks[t2[0]]
                else:
                    t3 = self.pop_scanner(self.tasksetHumanBound)
                    if len(t3)>0:
                        comm = self.tasksetHumanBound.tasks[t3[0]]
                    else:
                        t4 = self.task_available(self.tasksetTotal)
                        if len(t4)>0 and self.observation.action_h[-1]>=self.env.N_BIN:
                            for t in t4:
                                if command.isempty():
                                    comm = self.tasksetTotal.tasks[t]
                                elif self.tasksetTotal.tasks[t].product!=command.product:
                                    comm = self.tasksetTotal.tasks[t]
                                    
                                    
        elif assign_mode == 'HC':
            if comm.isempty():
                if self.tasksetHumanBound.ntask>3:
                    # 查询是否有可执行的 SCANNER 2 RECYCLE 任务
                    t1 = self.pop_scanner2recycle(self.tasksetHumanBound)
                    if len(t1)>0:
                        comm = self.tasksetHumanBound.tasks[t1[0]]
                    else:                  
                        t2 = self.pop_pod(self.tasksetHumanBound)
                        if len(t2)>0:
                            comm = self.tasksetHumanBound.tasks[t2[0]]
                        else:
                            t3 = self.pop_scanner(self.tasksetHumanBound)
                            if len(t3)>0:
                                comm = self.tasksetHumanBound.tasks[t3[0]]
                            else:
                                t4 = self.task_available(self.tasksetTotal)
                                if len(t4)>0 and self.observation.action_h[-1]>=self.env.N_BIN:
                                    for t in t4:
                                        if command.isempty():
                                            comm = self.tasksetTotal.tasks[t]
                                        elif self.tasksetTotal.tasks[t].product!=command.product:
                                            comm = self.tasksetTotal.tasks[t]
        
        action = self.format_action_out(command=command,comm=comm)
        # self.action = action
        self.reassign_flag = False          # 重置 重规划flag
        
        # time_end = time.time()
        # self.timebound -= time_end-time_begin
        # self.timebound = np.maximum(self.timebound,0)
        
        return action
    
    
    ## 检查 observation 是否合法
    def check_observation(self,obser:Observation):
        # 所有物品位置
        p_pos = obser.product_position
        # 如果物品位置中包含未探测位置，则非法
        if not all(np.sum(p_pos,axis=1)==1):
            return False
        else:
            return True
    
    ## 查询是否完成所有任务
    @property
    def done(self):
        if self.tasksetTotal.isempty():
            return True
        return False
    
    ## 形式化 action_h,action_r
    def format_action(self,action):
        if self.env.isPod(action[0]):
                actiontype = "POD2SCANNER"
        else:
            if self.env.isOrder(action[1]):
                actiontype = "SCANNER2ORDER"
            elif self.env.isRecycle(action[1]):
                actiontype = "SCANNER2RECYCLE"
        action_formatted = TaskAction(tasktype=actiontype,position_from=action[0],position_to=action[1],product=action[2])    
        
        return action_formatted


    ## format action
    def format_action_out(self,command,comm):
        action = np.array([self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN,self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN])
        print(f'format_action_out: comm:{comm.tasktype},{comm.position_from},{comm.position_to}')
        if command.isempty():
            action[0] = self.env.N_POSITION
            action[1] = self.env.N_POSITION
            action[2] = self.env.N_BIN
        elif command.tasktype=="POD2SCANNER":
            action[0] = command.position_from
            available_scanner = self.query_scanner_available()
            action[1] = available_scanner[0]
            action[2] = command.product
        elif command.tasktype=="SCANNER2ORDER" or command.tasktype == "SCANNER2RECYCLE":
            action[0] = command.position_from
            action[1] = command.position_to
            action[2] = command.product
            
        if comm.isempty():
            action[3] = self.env.N_POSITION
            action[4] = self.env.N_POSITION
            action[5] = self.env.N_BIN
        elif comm.tasktype=="POD2SCANNER":
            action[3] = comm.position_from
            available_scanner = self.query_scanner_available()
            action[4] = available_scanner[0]
            action[5] = comm.product
        elif comm.tasktype=="SCANNER2ORDER" or comm.tasktype == "SCANNER2RECYCLE":
            print(f'format_action_out: last formation')
            action[3] = comm.position_from
            action[4] = comm.position_to
            action[5] = comm.product
        return action.tolist()
    
    ## get_human_task_bound
    def get_human_task_bound(self):
        
        ## 1. init
        timebound = self.timebound
        weightsHuman = self.assignment[:,1].copy()      # 分配参数上界
        muRobot = self.tasksetTotal.timeparam[:,0]
        muHuman = self.tasksetTotal.timeparam[:,2]
        N = weightsHuman.size
        
        ## 2. problem definition
        ## 目标：集合内任务数量
        ## 约束1：bound 集合应在 human 任务集合内
        ## 约束2：在 bound 集合分配结果下，满足 timebound 约束
        ## 约束3：分配权重在{0,1}内
        weightsHumanLowbound = cp.Variable(N,boolean=True)
        obj = cp.Minimize(np.ones(N)@weightsHumanLowbound)
        constraints = [weightsHumanLowbound-weightsHuman<=np.zeros(N),
                    cp.maximum(muRobot@(1-weightsHumanLowbound),muHuman@weightsHumanLowbound)<=timebound,
                    weightsHumanLowbound>=0]
        prob = cp.Problem(obj,constraints)
        
        ## 3. solve and format results
        prob.solve()
        ## 若 bound 下无解，说明已经无法满足 timebound 约束，令 bound 集合为 human 任务集合
        if prob.status == 'infeasible':
            print("get_human_task_bound: problem is infeasible")
            self.tasksetHumanBound = self.tasksetTotal.assignment2taskset(self.assignment[:,1])
            return self.assignment
        ## 若 bound 有解，返回该 bound 解
        else:
            lowbound = weightsHumanLowbound.value
            self.tasksetHumanBound = self.tasksetTotal.assignment2taskset(lowbound)
            assignmentBound = np.concatenate(((1-lowbound).reshape((-1,1)),lowbound.reshape((-1,1))),axis=1)
            expetation = np.maximum(np.dot(lowbound,muHuman),np.dot((1-lowbound),muRobot))

        return assignmentBound, expetation

    ## optimize assignment
    def optimize_assign(self,nSamples=1e3,beta=.99):
        
        ## init
        nSamples = int(nSamples)
        timebound = self.timebound
        
        ## 1. query time params of tasksetTotal
        # self.tasksetTotal.set_time_param(self.taskEnv)
        muRobot = self.tasksetTotal.timeparam[:,0]
        sigmaRobot = self.tasksetTotal.timeparam[:,1]
        muHuman = self.tasksetTotal.timeparam[:,2]
        sigmaHuman = self.tasksetTotal.timeparam[:,3]
        
        # available task index
        available_index = self.task_available(self.tasksetTotal)
        task_index = np.zeros(len(self.tasksetTotal.tasks),dtype=int)
        for i in available_index:
            task_index[i] = 1
        
        ## 2. generate time data 
        timeDataRobot = np.random.multivariate_normal(muRobot,np.diag(sigmaRobot),size=int(nSamples))
        timeDataHuman = np.random.multivariate_normal(muHuman,np.diag(sigmaHuman),size=int(nSamples))
        
        ## 3. variable
        N = int(self.tasksetTotal.ntask)
        weightsRobot = cp.Variable(N)
        weightsHuman = cp.Variable(N)
        u = cp.Variable(int(nSamples))
        alpha = cp.Variable(1)
        
        ## 4. objective
        obj = cp.Minimize(alpha+1/nSamples/(1-beta)*(np.ones(nSamples) @ u))

        ## 5. constraints
        constraints = [u>=0,
                    u-cp.maximum(timeDataHuman@weightsHuman,timeDataRobot@weightsRobot)+alpha>=0,
                    weightsHuman+weightsRobot==np.ones(N),
                    weightsHuman>=0,weightsRobot>=0,
                    muHuman@weightsHuman<=timebound,muRobot@weightsRobot<=timebound,
                    task_index@weightsRobot>=1]

        ## 6. problem and solve it
        prob = cp.Problem(obj,constraints)
        prob.solve()
        probtype = 1
        
        if prob.status=='infeasible':
            constraints = [u>=0,
                    u-cp.maximum(timeDataHuman@weightsHuman,timeDataRobot@weightsRobot)+alpha>=0,
                    weightsHuman+weightsRobot==np.ones(N),
                    weightsHuman>=0,weightsRobot>=0,
                    muHuman@weightsHuman<=timebound,muRobot@weightsRobot<=timebound]
            prob = cp.Problem(obj,constraints)
            prob.solve()
            probtype = 2
            if prob.status == 'infeasible':
                obj = cp.Minimize(cp.maximum(muHuman@weightsHuman,muRobot@weightsRobot))
                constraints = [weightsHuman+weightsRobot==np.ones(N),
                    weightsHuman>=0,weightsRobot>=0]
                prob = cp.Problem(obj,constraints)
                prob.solve()
                probtype = 3    
        
        r_ = weightsRobot.value
        h_ = weightsHuman.value
    
        r = cp.Variable(N,boolean=True)
        obj = cp.Minimize(cp.norm2(r-r_))
        if probtype<3:
            constraints = [muHuman@(1-r)<=timebound,muRobot@r<=timebound]
        else:
            constraints = []
        prob = cp.Problem(obj,constraints)
        prob.solve(solver=cp.CPLEX)
        
        ## 7. format solutions  ## TODO: 当原有的optimize函数无解的时候如何处理，这里出错是因为所有的物品robot都可以抓
        r = np.around(r.value)
        h = 1-r
        self.assignment = np.concatenate((r.reshape((-1,1)),h.reshape((-1,1))),axis=1)
        
        ## 8. set tasksetRobot and tasksetHuman
        self.tasksetRobot = self.tasksetTotal.assignment2taskset(self.assignment[:,0])
        self.tasksetHuman = self.tasksetTotal.assignment2taskset(self.assignment[:,1])
        
        pass
        
    ## 查询有效的 SCANNER 2 RECYCLE 任务
    def pop_scanner2recycle(self,taskset:TaskSet):
        task_scanner2recycle = taskset.where_tasks_index('SCANNER2RECYCLE')
        if len(task_scanner2recycle)==0:
            return []
        else:
            t_idx = []
            for t in task_scanner2recycle:
                if self.env.isScanner(taskset.tasks[t].position_from):
                    t_idx.append(t)
            return []


    ## 查询有效的 P2S 任务
    def pop_pod2scanner(self):
        tasksid = self.tasksetRobot.where_tasks_index("POD2SCANNER")
        if len(tasksid)==0:
            return []
        scanner_available = self.query_scanner_available()
        if len(scanner_available)==0:
            return None
        else:
            return tasksid[0]


    ## 查询有效的 S2O 任务
    def pop_scanner2order(self):
        tasksid = self.tasksetRobot.where_tasks_index("SCANNER2ORDER")
        if len(tasksid)==0:
            return []
        else:
            for t in tasksid:
                p = self.tasksetRobot.tasks[t].product
                # 如果 p 的位置不在 robot、human 中，则返回该任务 id
                if np.nonzero(self.observation.product_position[p,:])[0][0]<self.env.N_POSITION:
                    return t
            return []
        
    ## 查询在 SCANNER 中的任务
    def pop_scanner(self,taskset:TaskSet):
        tasks = taskset.tasks
        task_idx = []
        for t_idx in range(taskset.ntask):
            if tasks[t_idx].tasktype == "SCANNER2ORDER" or tasks[t_idx].tasktype=="SCANNER2RECYCLE":
                if self.env.isScanner(tasks[t_idx].product):
                    task_idx.append[t_idx]
                    
        return task_idx

    ## 查询有效的 POD 2 SCANNER 任务
    def pop_pod(self,taskeset:TaskSet):
        free_scanner = self.query_scanner_available()
        if len(free_scanner)>0:
            return []
        for t in taskeset.tasks:
            if t.tasktype=="POD2SCANNER":
                return t
        return []

    ## 查询 product 相应的 order
    def query_order_from_product(self,product=None):
        if product==None:
            return []
        return np.nonzero(self.order[product,:])[0]
    
    ## 查询 scanner 中的 product
    def query_product_in_scanner(self):
        product_position = self.observation.product_position[:,self.env.SCANNER_INDEX_FROM:self.env.SCANNER_INDEX_TO]
        return np.nonzero(product_position)


    ## 查询所有 product 的位置，以向量形式表示
    def query_product_position(self):
        for p in range(self.env.N_BIN):
            if np.sum(self.observation.product_position[p,:])==0:
                self.observation.product_position[p,:] = np.zeros((self.env.NALL_POSITION,1),dtype=int)
                self.observation.product_position[p,p] = 1
        return np.nonzero(self.observation.product_position)[1]
    
    ## 查询可用的 scanner area，返回可用的 SCANNER 的位置
    def query_scanner_available(self):
        product_position_in_scanner = self.observation.product_position[:,self.env.SCANNER_INDEX_FROM:self.env.SCANNER_INDEX_TO]
        scanner_occupied = np.minimum(np.sum(product_position_in_scanner,axis=0),1,dtype=int)
        toscanner = 0
        act_h = self.observation.action_h
        act_r = self.observation.action_r
        
        ## 若 HUMAN ongoing 任务为 POD2SCANNER，增加 occupied scanner 数量
        if act_h[-1]>=0 and act_h[-1]<self.env.N_BIN:
            toscanner += 1
        
        ## 若 occupied 数量超过 SCANNER 数量，则返回空集
        if np.count_nonzero(scanner_occupied)>=(self.env.N_SCANNER-toscanner):
            return []
        else:
            # 将 robot 的 ongoing 的 POD2SCANNER 任务的目标位置至为 occupied
            if len(act_r)>0 and act_r[0]<self.env.BIN_INDEX_TO and act_r[0]>=0:
                scanner_occupied[act_r[1]-self.env.N_BIN] = 1
            return np.nonzero(scanner_occupied-1)[0]+self.env.N_BIN
        
    ## 查询任务是否 robot 可执行
    def query_robot_do_task(self,task:PickPlaceTask):
        product = task.product
        return self.env.query_cargo_type(product)
        
    ## 新建线程用于接收 topic 数据 
    def spin_thread(self):
        rospy.spin()

    # 发送 robot 下一任务至 topic
    def send_command_to_robot(self,command):
        # self.command_msg.data = int(command)
        self.command_msg.data = command
        self.command_pub.publish(self.command_msg)
        pass
        
    # 发送 human 下一任务至 topic
    def send_comm_to_human(self,comm):
        
        rospy.wait_for_service('Mp3_player')
        req = Mp3Request()
        req.cmd = comm
    
        mp3_client = rospy.ServiceProxy('Mp3_player', Mp3)
        resp = mp3_client(req)

    @property
    def timeout(self,timebound=None):
        if timebound is None:
            timebound = self.timebound

        timeparam = np.array(self.tasksetTotal.timeparam)
        if timeparam.size==0:
            return False
        else:
            muR = np.array(self.tasksetTotal.timeparam[:,0])
            muH = np.array(self.tasksetTotal.timeparam[:,2])
        
        if muR.size==0 or muH.size==0:                  # 若 taskset 时间参数为空，表明未开始规划
            return False
        
        if len(self.assignment)==0:
            return False
        
        weightR = np.array(np.round(self.assignment[:,0]),dtype=int)
        weightH = np.array(np.round(self.assignment[:,1]),dtype=int)
        if muR.size != weightR.size or muH.size != weightH.size:
            return False
        timeExpetation = np.maximum(np.dot(muR,weightR),np.dot(muH,weightH))
        if timeExpetation>timebound:
            return True
        return False
    
    
    ## 查询任务集合内可执行的任务
    def task_available(self,taskset:TaskSet):
        # 如果 taskset 为空，则返回空
        if len(taskset.tasks)==0:
            return []
        
        free_scanner = self.query_scanner_available()
        product_position = self.query_product_position()
        task_idx = []
        for t_idx in range(len(taskset.tasks)):
            task = taskset.tasks[t_idx]
            if task.tasktype == "POD2SCANNER":
                if len(free_scanner)>0:
                    task_idx.append(t_idx)
            if task.tasktype == "SCANNER2ORDER" or task.tasktype == "SCANNER2RECYCLE":
                if self.env.isScanner(product_position[task.product]):
                    task_idx.append(t_idx)
        return task_idx
    

    ## 更新 observation
    def update_observation(self,data):
        
        # 如果更新的数据类型不是 Observation 类，则更新观测方式为 topic，需要根据 ongoing task 更新观测值中的位置信息
        if type(data)!=Observation:
            position = np.array(data.position.data,dtype=int).reshape((self.env.N_BIN,self.env.N_POSITION+3))
            ## 若存在观测值中物品同时存在两个位置，则观测值非法
            if not all(np.sum(position,axis=1)==1):
                return False
            action_obser = np.array(data.action.data,dtype=int)
            
            action_h = action_obser[3:6]
            action_r = action_obser[0:3]
            # 因为观测值中的物品位置只包含摄像头数据，因此根据收到的当前行为对物品位置数据进行更新
            # 1. 摄像头包含的物品数量
            n_p_fromcamera = np.sum(position[:])
            # 2. 若摄像头数量小于物品数量，则查看 robot action 是否有效
            if n_p_fromcamera<self.env.N_BIN:
                # 若 robot 中的物品有效，则将对应物品位置至为 robot
                if action_h[-1]>=0 and action_h[-1]<self.env.N_BIN:
                    position[action_h[-1],self.env.HUMAN_INDEX]=1
                    # camera 中的物品数量+1
                    n_p_fromcamera += 1
            else:
                # robot 的 action 非法 
                action_h=[self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN]
                
            # 3. 若摄像头数量小于物品数量，则查看 human action 是否有效
            if n_p_fromcamera<self.env.N_BIN:
                if action_r[-1]>=0 and action_r[-1]<self.env.N_BIN:
                    position[action_r[-1],self.env.ROBOT_INDEX]=1
                else:
                    action_r=[self.env.NALL_POSITION,self.env.NALL_POSITION,self.env.N_BIN]
            observation = Observation(product_position=position,action_h=action_h,action_r=action_r)
        ## 如果为 sim、test 模式，则直接采用观测值
        else:
            observation = data

        observation_change_flag = not bool(self.observation.identical(observation))     # 判断观测值是否相同
        # 若函数输入的观测值不同则更新观测值，并设置 reassignment flag 为 true
        if observation_change_flag or self.reassign_flag:                             # 如果观测值进行了更新
            # 检查 observation 是否有效，无效则不进行规划
            if not self.check_observation(observation):
                print(f'update_observation: illegal observation...')
                return False
            self.reassign_flag = True                               # 设置 reassignment 为 True     
            self.observation.copy_from_observation(observation)     # 更新观测值
            self.update_taskset()
            
            action = self.assign()       # 任务分配解算
            print(f'update_observation: return of assign: {action}')
            if action[2]<self.env.N_BIN and action[2]>=0:
                print(f'action output: {action}')
                self.send_command_to_robot(action[0:3])          # 发送命令至 robot
                self.action[0:3] = action[0:3]
            # 仅当解算的新的 communication 不同则发送新的 communication
            if len(action)>3:
                if action[5]<self.env.N_BIN and action[5]>=0:
                    if np.any(action[3:6]!=self.action[3:6]):
                        self.action[3:6] = action[3:6]
                        self.send_comm_to_human(action[3:6])         # 发送 communication 至 human
                        # print(f'update_observation: send to human: {action[3:6]}')
                
                    ## TODO: 或调用 communication service


            
        else:
            # 超时需重新优化权重
            if self.timeout:
                
                self.reassign_flag = True
            else:
                self.reassign_flag = False                          # 否则重置 重规划 Flag

        
        return bool(observation_change_flag)


    ## 根据 self.observation 更新任务集合
    def update_taskset(self):
        product_position = self.query_product_position()                                # 全体 product 位置
        self.tasksetTotal.reset()                                                 # 清空 tasksetTotal 列表
        tasklist = []                                                                   # 初始化 tasklist
        action_h = self.observation.action_h                                            # 当前 human action
        action_r = self.observation.action_r                                            # 当前 robot action
        
        # 若 product 位置在 POD 且需要移动到订单、或在 scanner 中，则增加相应 task 至 tasklist中
        order_new = self.observation.product_position[:,self.env.ORDER_INDEX_FROM:self.env.ORDER_INDEX_TO]
        # 当前 order 临时变量
        order_ = np.maximum(self.order-order_new,0)        
        
        # 若某个物品位置消失，报错
        if product_position.size<self.env.N_BIN:
            raise Exception(f'Products disappear...')
        
        # 查询所有的物品，根据物品现在的位置、是否在 ROBOTO、HUMAN 中，生成总任务集合
        for p in range(self.env.N_BIN):

            # 查询 p 位置
            p_position = product_position[p]                                                
            order_idx = np.where(order_[p,:]>0)[0]
            if len(order_idx)>0:
                order_position = order_idx[0]+self.env.N_BIN+self.env.N_SCANNER

            # products 在 HUMAN、ROBOT 中
            # 如果 robot 中的任务为 p
            if self.env.isRobot(p_position):
                continue
                # if type(action_r)!=TaskAction:
                #     action_r = self.format_action(action_r)
                # # 如果 robot 任务为 P2S
                # if action_r.tasktype == "POD2SCANNER":
                #     # 如果 robot 任务合法，增加 S2O 任务，否则增加 S2R 任务
                #     if len(order_idx)>0:
                #         order_idx = order_idx[0]
                #         tasklist.append(PickPlaceTask(tasktype="SCANNER2ORDER",
                #                                     position_from=action_r.position_to,
                #                                     position_to=order_position,
                #                                     product=p))
                #         order_[p,order_idx] -= 1
                #         order_[p,order_idx] = np.maximum(order_[p,order_idx],0)
                #     else:
                #         tasklist.append(PickPlaceTask(tasktype="SCANNER2RECYCLE",
                #                                     position_from=action_r.position_to,
                #                                     position_to=self.env.RECYCLE_INDEX,
                #                                     product=p))
                # # 如果 robot 执行 S2O 任务，更新临时 order 变量
                # elif action_r.tasktype == "SCANNER2ORDER":
                #     order_idx = order_idx[0]
                #     order_[p,order_idx] -= 1
                #     order_[p,order_idx] = np.maximum(order_[p,order_idx],0)
                    
            # 如果 human 中的任务为 p
            if self.env.isHuman(p_position):
                continue
                # # 如果 human 执行 P2S 任务
                # if type(action_h)!=TaskAction:
                #     action_h = self.format_action(action_h)
                # if action_h.tasktype == "POD2SCANNER":
                #     # 如果 human 任务合法，增加 S2O 任务，否则增加 S2R 任务
                #     if len(order_idx)>0:
                #         order_idx = order_idx[0]
                #         tasklist.append(PickPlaceTask(tasktype="SCANNER2ORDER",
                #                                         position_from=action_h.position_to,
                #                                         position_to=order_position,
                #                                         product=p))
                #         order_[p,order_idx] -= 1
                #         order_[p,order_idx] = np.maximum(order_[p,order_idx],0)
                #     else:
                #         tasklist.append(PickPlaceTask(tasktype="SCANNER2RECYCLE",
                #                                     position_from=action_h.position_to,
                #                                     position_to=self.env.RECYCLE_INDEX,
                #                                     product=p))
                # # 如果 human 执行 S2O 任务，更新临时 order 变量
                # elif action_h.tasktype == "SCANNER2ORDER":
                #     order_idx = order_idx[0]
                #     order_[p,order_idx] -= 1
                #     order_[p,order_idx] = np.maximum(order_[p,order_idx],0)

            # products 在 SCANNER 中
            if self.env.isScanner(p_position):
                # 如果 order 不需要则增加 S2R 任务
                if len(order_idx)==0:
                    tasklist.append(PickPlaceTask(tasktype="SCANNER2RECYCLE",
                                                position_from=p_position,
                                                position_to=self.env.RECYCLE_INDEX,
                                                product=p))
                # 如果 order 需要，则增加 S2O 任务
                else:
                    order_idx = order_idx[0]
                    tasklist.append(PickPlaceTask(tasktype="SCANNER2ORDER",
                                                    position_from=p_position,
                                                    position_to=order_position,
                                                    product=p))
                    order_[p,order_idx] -= 1
                    order_[p,order_idx] = np.maximum(order_[p,order_idx],0)

            # product 在 POD 中
            if self.env.isPod(p_position) and len(order_idx)>0:
                order_idx = order_idx[0]
                tasklist.append(PickPlaceTask(tasktype='POD2SCANNER',
                                                position_from=p_position,
                                                position_to=[],
                                                product=p))
                tasklist.append(PickPlaceTask(tasktype="SCANNER2ORDER",
                                                position_from=[],
                                                position_to=order_position,
                                                product=p))
                
                order_[p,order_idx] -= 1
                order_[p,order_idx] = np.maximum(order_[p,order_idx],0)
        
        # 更新 taskset 时间参数
        muRobot,sigmaRobot,muHuman,sigmaHuman = [],[],[],[]
        if len(tasklist)>0:
            for t in tasklist:
                product = t.product
                cargotype = self.env.query_cargo_type(productid=product)
                muR,sigmaR = self.env.query_time_param(True,t.tasktype,cargotype)
                muH,sigmaH = self.env.query_time_param(False,t.tasktype,cargotype)
                muRobot = np.append(muRobot,muR)
                muHuman = np.append(muHuman,muH)
                sigmaRobot = np.append(sigmaRobot,sigmaR)
                sigmaHuman = np.append(sigmaHuman,sigmaH)
            timeParamRobot = np.concatenate((muRobot.reshape((-1,1)),sigmaRobot.reshape((-1,1))),axis=1)
            timeParamHuman = np.concatenate((muHuman.reshape((-1,1)),sigmaHuman.reshape((-1,1))),axis=1) 
            timeparam = np.concatenate((timeParamRobot,timeParamHuman),axis=1)
            
            # 更新 tasksetTotal
            self.tasksetTotal.set_tasks(tasks=tasklist,timeparam=timeparam)
        else:
            self.tasksetTotal.set_tasks(tasks=[])
        
        pass    
        

## 真实环境
def main_real():
    # 初始化 planner
    config_file = '/home/wy/t_ws/src/hm_cooperation/scripts/data/env_params.json'             ##! TODO: 
    env = TaskEnv(file_name=config_file)
    planner = TaskAssignment(env=env)
    
    # 每次循环进行解算并发送命令
    rate = rospy.Rate(10)
    while True:
        # print(f'update_observation: time bound: {planner.timebound}')

        # time_begin = time.time()
        # rospy.sleep(1)
        # time_end = time.time()
        # planner.timebound -= time_end-time_begin
        # planner.timebound = np.maximum(planner.timebound,0)
        if planner.done:
            break
        
'''    
## 仿真环境    
def main_sim(): 
    
    ##=============================================================================
    ## 1. real env
    env_real = OrderPickingEnv()
    env_real.reset()
    env_real.render()
    
    ## 3. assigner
    planner = TaskAssignment(env=TaskEnv("env_params.json"),timebound=80)
    
    ## 4. 交互
    while True:
        # 4.1 receive and upate observation at time t
        observation_ = env_real._get_total_obs()
        

        # 4.2 assign actions
        action_ = planner.assign(observation=observation_)
        
        # 4.3 real env step and render
        env_real.step(action_)
        env_real.render(mode='human',action=action_)

## 测试环境
def main_test():
    
    # 读取环境配置参数文件
    config_file = 'data/env_params.json'
    env = TaskEnv(file_name=config_file)
    
    # 读取测试参数文件
    product_position = np.zeros((12,24))
    p = pd.read_excel("data/test_product_position_5.xlsx").values.ravel()
    for p_ in range(12):
        product_position[p_,p[p_]]=1
    action_h = pd.read_excel("data/test_action_h5.xlsx").values.ravel()
    action_r = pd.read_excel("data/test_action_r5.xlsx").values.ravel()
    obser = Observation(product_position=product_position,
                        action_h=action_h,
                        action_r=action_r)
    # planner
    planner = TaskAssignment(env=env)
    

    action = planner.assign(observation=obser)
    print(f'action is {action}')
'''

if __name__ == '__main__':
    main_real()