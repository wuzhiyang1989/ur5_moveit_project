from logging import raiseExceptions
import numpy as np
import json

##===========================================================================================
##
## CLASS OBSERVATION
## 
##===========================================================================================
class Observation():
    def __init__(self,product_position=[],action_h=[],action_r=[],order=[]) -> None:
        self.product_position = product_position        # 所有 product 的位置, shape(n_product,)
        self.action_h = action_h                        # Human action
        self.action_r = action_r                        # Robot action
        pass
    
    ## 检测是否与另一 observation 相同
    def identical(self,obs):
        if np.all(self.product_position == obs.product_position):
            return True
        return False
        
    ## 从另一 observation <type Observation> 复制观测值
    def copy_from_observation(self,observation) -> None:
        self.product_position = np.array(observation.product_position,dtype=int)    # copy 所有product位置信息
        self.action_h = observation.action_h
        self.action_r = observation.action_r
        pass


##===========================================================================================
##
## CLASS TaskEnv is base class to describe the sorting envrionment
##
##===========================================================================================
class TaskEnv:
    def __init__(self,file_name) -> None:
        
        f = open(file_name,encoding='utf-8')        # 打开任务环境参数文件
        data = json.load(f)                         # 加载观测数据至 data
        
        # Env params
        self.N_BIN = data['N_BIN']                                  # product 数量
        self.N_SCANNER = data['N_SCANNER']                          # scanner 数量
        self.N_ORDER = data['N_ORDER']                              # order 数量
        self.TIME_BOUND = data['TIME_BOUND']                        # 任务总时间
        self.ORDER = data['ORDER']                                  # 任务列表
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
        
        
        pass
    
    ##---------------------------------------------------------------------------------------
    ## 从位置编号查询 scanner 相应序号
    ##---------------------------------------------------------------------------------------
    def query_scanner_id(self,pos):
        pos = np.array(pos).squeeze().ravel()
        scanner_id = []
        for p_ in pos:
            if self.query_position(p_,'SCANNER').size <=0:
                raise ValueError('Input "pos" is not in SCANNER')
            else:
                scanner_id.append(p_-self.N_BIN)
        return np.array(scanner_id,dtype=int).squeeze()
    
    def query_position(self,pos:np.ndarray,where_str):
        if type(where_str)==str:
            where_str = [where_str]
        pos = pos.squeeze().ravel()
        if len(where_str) != pos.size:
            raise ValueError('The sie of "where_str" and "pos" are supposed to be same')
        position_ = []
        for i in range(pos.size):
            if where_str[i] == 'POD':
                position_.append(int(pos[i]))
            elif where_str[i] == 'SCANNER':
                position_.append(int(pos[i])+self.N_BIN)
            elif where_str[i] == 'ORDER':
                position_.append(int(pos[i])+self.N_BIN+self.N_SCANNER)
        return np.array(position_,dtype=int).squeeze()

    ## query_time_param
    def query_time_param(self,isRobot:bool,tasktype:str,cargotype:bool):
        cargotypeindex = 0 if cargotype else 1
        if tasktype == 'POD2SCANNER':
            tasktypeindex = 0
        elif tasktype == 'SCANNER2ORDER':
            tasktypeindex = 1
        elif tasktype == 'SCANNER2RECYCLE':
            tasktypeindex = 2
        timeparam = self.timeinfo[0,tasktypeindex,cargotypeindex] if isRobot else self.timeinfo[1,tasktypeindex,cargotypeindex]
        return tuple(timeparam)
        
    ## query_cargo_type
    def query_cargo_type(self,productid):
        return self.cargoinfo[productid]
    
    ## pos 是否在 POD 内
    def isPod(self,pos):
        return bool(pos>=self.BIN_INDEX_FROM and pos<self.BIN_INDEX_TO)
    
    ## pos 是否在 SCANNER 内
    def isScanner(self,pos):
        return bool(pos>=self.SCANNER_INDEX_FROM and pos<self.SCANNER_INDEX_TO)
    
    ## pos 是否在 ORDER 内
    def isOrder(self,pos):
        return bool(pos>=self.ORDER_INDEX_FROM and pos<self.ORDER_INDEX_TO)
    
    ## pos 是否在 Robot 中
    def isRobot(self,pos):
        return pos == self.ROBOT_INDEX
    
    ## pos 是否在 Human 中
    def isHuman(self,pos):
        return pos == self.HUMAN_INDEX
    
    ## pos 是否在 Recycle 中
    def isRecycle(self,pos):
        return pos == self.RECYCLE_INDEX


##===========================================================================================
##
## CLASS PickPlaceTask is base class for sorting tasks.
##
##===========================================================================================
class PickPlaceTask:
    def __init__(self,tasktype='',position_from=[],position_to=[],product=[]) -> None:
        self.tasktype = tasktype
        self.position_from = position_from
        self.position_to = position_to
        self.product = product
        pass
    def isempty(self):
        if len(self.tasktype)>0:
            return False
        else:
            return True
    
##===========================================================================================
##
## CLASS ACTION
##
##===========================================================================================
class TaskAction(PickPlaceTask):
    def __init__(self,tasktype='',position_from=[],position_to=[],product=[]):
        super(TaskAction, self).__init__(tasktype,position_from,position_to,product)
        pass
    def isempty(self):
        if len(self.tasktype)==0:
            return True
        else:
            return False


##===========================================================================================
##
## CLASS TASKSET
##
##===========================================================================================
class TaskSet:
    ## initialization
    def __init__(self,tasks=[],timeparam=[]) -> None:
        self.tasks = tasks                          # 集合内的任务, list[PickPlaceTask]
        self.timeparam = np.array(timeparam)        # 集合内任务的时间参数, ndarray
        # if len(tasks)!=timeparam.shape[0]:
        #     raiseExceptions('Init TaskSet error: number of tasks should equal to timeparam')
        # pass
    
    ## 清空当前任务列表
    def reset(self):
        self.tasks.clear()
        self.timeparam=np.array([])
    
    ## pop_task_index 按照指定的任务类型 'tasktype' 搜索集合内的现有任务
    def pop_task_index(self,tasktype:str):
        for t in range(len(self.tasks)):
            if self.tasks[t].tasktype == tasktype:
                return t
        return []
            
    ## isempty 判断任务集合是否为空
    def isempty(self):
        if len(self.tasks)<=0:
            return True
        else:
            return False
    
    ## 根据新的 assignment 生成任务子集
    def assignment2taskset(self,assignment=np.array([])):
        assignment = np.array(assignment,dtype=int)
        if assignment.size==0:
            return TaskSet(self.tasks,self.timeparam)
        tasklist = []
        timeparamlist = []
        for i in range(assignment.size):
            if assignment[i]>0:
                tasklist.append(self.tasks[i])
                timeparamlist.append(list(self.timeparam[i,:]))
        return TaskSet(tasklist,timeparam=timeparamlist)
    
    ## 集合内 tasks 数量
    @property
    def ntask(self):
        if self.tasks==[]:
            return 0
        else:
            return int(len(self.tasks))
        
    ## 设置任务集合参数
    def set_time_param(self,timeparam):
        timeparam = np.array(timeparam)
        if timeparam.shape!=(len(self.tasks),4):
            raiseExceptions("Shape of TaskSet.set_time_param input must equal to (TaskSet.tasks,4)")
        else:
            self.timeparam = timeparam

    ## 设置任务集合
    def set_tasks(self,tasks,timeparam=[]):
        self.tasks.clear()
        self.tasks = tasks
        if len(timeparam)>0:
            self.set_time_param(timeparam)
            
    ## 返回指定类型的任务类型的 index
    def where_tasks_index(self,tasktype):
        index = []
        for i in range(len(self.tasks)):
            if self.tasks[i].tasktype==tasktype:        # 如果 taskset 中的 task 类型和指定类型一致
                index.append(i)                             # 增加该 task id 到 index 中
        return index

