#usr/bin/python3
# -*- coding: utf-8 -*-

import serial
import time
from time import sleep
#import surface
import robotcontrol

"""大寰电动夹爪指令"""
dh_linkcheck_buf = [0x55, 0xAA, 0x04, 0xE1, 0x00, 0x00, 0x00, 0xE4]
dh_projectstart_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x01, 0xE8]
dh_grabstart_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x02, 0xE9]
dh_prjectend_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x03, 0xEA]
dh_mutexcaptrack_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0xF1, 0xD8] # 人手检测 互电容电容追踪
dh_handdetect_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0xF2, 0xD9] # 上位机人手检测
dh_arrivepos_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0xF3, 0xDA] # 抓取矿泉水瓶,已到达指定位置,可倒水
dh_sampletrial_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0xF4, 0xDB] # 发送抓取试验样品

dh_moveleft_buf = [0x55, 0xAA, 0x04, 0xE3, 0x00, 0x00, 0x02, 0x00, 0x06, 0xEE] #左移完成状态量
dh_moveright_buf = [0x55, 0xAA, 0x04, 0xE3, 0x00, 0x00, 0x02, 0x00, 0x07, 0xEF] #右移完成状态量

class uart_surpara():
    def __init__(self):
        self.sequence = 0
    
    def change(self, para):
        self.sequence = para
        return self.sequence

class uart_queue():
    """队列"""
    def __init__(self, size):
        # 初始化
        self.data = [] #数据列表存储
        self.size = size
        self.head = 0
        self.tail = 0
    
    def empty(self):
        """判断队列是否为空"""
        if self.head == self.tail:
            return True
        else:
            return False
        
    def full(self):
        """判断队列是否已满"""
        if (self.tail - self.head) == self.size:
            return True
        else:
            return False
            
    def join(self, content):
        """入队列"""
        if self.full():
            print("队列已满,不能进入队列")
        else:
            self.data.append(content)
            self.tail = self.tail + 1
    
    def out(self):
        """出队列"""
        if self.empty():
            print("队列已空,没有元素出队列")
        else:
            datamid = self.data[self.head]
            self.data[self.head] = "null"
            self.head = self.head + 1
            return datamid
            
    def sayhead(self):
        """当前队首下标位置"""
        print("当前队首下标位置是: " + str(self.head))

    def saytail(self):
        """当前队尾下标位置"""
        print("当前队尾下标位置是: " + str(self.tail))

    def length(self):
        """返回大小"""
        return (self.tail - self.head)

def uart_init():
    global dh_uart_ser
    global dh_uartqueue

    # 创建数据列表
    dh_uartqueue = uart_queue(120)
    # 建立串口连接
    dh_uart_ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

    """
    uart_tryconnect_num = 3
    while uart_tryconnect_num:
        dh_uart_ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
        if dh_uart_ser.is_open() == True:
            print("uart open success")
            break
        else:
            print("open failed")
        uart_tryconnect_num -=1
    """

def uart_close():
    """关闭串口"""
    dh_uart_ser.close()

def uart_send(serial, cmdbuf):
    while True:
        test_sendbuf = bytes(cmdbuf)
        serial.write(test_sendbuf)
        break

 
def uart_recv(serial):
        global data
        while True:
            data = serial.read(48)
            if data == '':
                continue
            else:
                break

            time.sleep(0.1)

        return data

def dh_uart_recv():
    """串口接收程序"""
    data = uart_recv(dh_uart_ser) 
    middatas=''.join(map(lambda x:('/x' if len(hex(x))>=4 else '/x0')+hex(x)[2:],data))
    new_datas=middatas[2:].split('/x') # 由于datas变量中的数据前两个是/x，所以用到切片工具
    if new_datas != ['']: 
        print(new_datas)
        while len(new_datas):
            dh_uartqueue.join(new_datas[0])
            new_datas.remove(new_datas[0])  

def dh_uart_send(sendflag):
    """串口发送程序"""
    if sendflag == 'LinkCheck':
        uart_send(dh_uart_ser, dh_linkcheck_buf) # 链路检测
    elif sendflag == 'ProjectStart':
        uart_send(dh_uart_ser, dh_projectstart_buf) # 项目开始
    elif sendflag == 'GrabStart':
        uart_send(dh_uart_ser, dh_grabstart_buf) # 抓取开始
    elif sendflag == 'ProjectEnd':
        uart_send(dh_uart_ser, dh_prjectend_buf) # 项目结束
    elif sendflag == 'MutexCapTrack':
        uart_send(dh_uart_ser, dh_mutexcaptrack_buf) # 人手检测 互电容追踪    
    elif sendflag == 'HandDetect':
        uart_send(dh_uart_ser, dh_handdetect_buf) # 人手检测   
    elif sendflag == 'ArrivePos':
        uart_send(dh_uart_ser, dh_arrivepos_buf) # 抓起矿泉水瓶,已到达指定位置,可二次检测滑动  
    elif sendflag == 'SampleTrial':
        uart_send(dh_uart_ser, dh_sampletrial_buf) # 除人手检测和抓取矿泉水瓶, 抓取其他样品, 发送只抓紧,不放松,不自动回到原位命令
    elif sendflag == 'MoveLeft':
        uart_send(dh_uart_ser, dh_moveleft_buf) # 左移完成      
    elif sendflag == 'MoveRight':
        uart_send(dh_uart_ser, dh_moveright_buf) # 左移完成         
        
    #time.sleep(0.1) 

                

def uart_listpos(datalist):
    """列表中指定元素出现的位置""" 
    return datalist.index('55')


def dh_uart_listdata_analysis():
    listrecv_databuf = [] 
    """列表数据解析"""
    if dh_uartqueue.length() >= 8:
        count_num = 4
        while count_num:
            listrecv_databuf.append(dh_uartqueue.out()) 
            count_num -= 1
        
        if listrecv_databuf[:3] == ['55', 'aa', '03']:
            if listrecv_databuf[3] == 'e1': # 链路检测
                count_num = 4
                while count_num:
                    listrecv_databuf.append(dh_uartqueue.out()) 
                    count_num -= 1

                del listrecv_databuf[:] #清空列表
                robotcontrol.dh_robot_topchangepos(0.3, 0.3, 3)
                print("Exe Back Point")
                dh_uart_send('ProjectEnd') # 发送任务结束
                print("Send Project End")
 
            elif listrecv_databuf[3] == 'e2': # 控制命令
                if dh_uartqueue.length() >= 6:
                    count_num = 6
                    while count_num:
                        listrecv_databuf.append(dh_uartqueue.out()) 
                        count_num -= 1
                    
                    if listrecv_databuf[8] == '01':
                        del listrecv_databuf[:] #清空列表
                        print("接收任务开始反馈")
                    elif listrecv_databuf[8] == '02':
                        del listrecv_databuf[:] #清空列表
                        print("接收抓取开始反馈")
                    elif listrecv_databuf[8] == '03':
                        del listrecv_databuf[:] #清空列表
                        print("接收任务结束反馈")  
                    elif listrecv_databuf[8] == '04': # 向左移动
                        del listrecv_databuf[:] #清空列表
                        dh_uart_send('MutexCapTrack')
                        print("Exe MutexCap Track")
                        robotcontrol.dh_robot_poschange('pos_y', 0.002, 0.015, 0.002) #0.12 1cm                        
                        print("机械臂向左移动")
                        dh_uart_send('MoveLeft') # 左移完成
                        dh_uart_send('ProjectStart') #抓取矿泉水瓶
                        print("Send Project Start") 
                    elif listrecv_databuf[8] == '05': # 向右移动
                        del listrecv_databuf[:] #清空列表
                        dh_uart_send('MutexCapTrack')
                        print("Exe MutexCap Track")
                        robotcontrol.dh_robot_poschange('pos_y', 0.002, 0.015, -0.002) #0.12 1cm                      
                        print("机械臂向右移动") 
                        dh_uart_send('MoveRight') # 右移完成
                        dh_uart_send('SampleTrial')
                        print("Send Sample Trial")
                    elif listrecv_databuf[8] == '10': # 回到原来位置
                        del listrecv_databuf[:] #清空列表
                        dh_uart_send('LinkCheck')
                        print("Send Back Point")
                    elif listrecv_databuf[8] == 'f4': # 接收到样品试验
                        del listrecv_databuf[:] #清空列表
                        print("Recv Sample Trial Cmd")
                        dh_uart_send('ProjectStart')
                        print("Send Project Start")                      


            elif listrecv_databuf[3] == 'e3': # 状态量
                if dh_uartqueue.length() >= 6:
                    count_num = 6
                    while count_num:
                        listrecv_databuf.append(dh_uartqueue.out()) 
                        count_num -= 1
                    
                    if listrecv_databuf[8] == '01':
                        del listrecv_databuf[:] #清空列表
                        print("任务开始状态量反馈")
                        dh_uart_send('GrabStart')
                        print("Send Grab Start")
                    elif listrecv_databuf[8] == '02':
                        del listrecv_databuf[:] #清空列表
                        print("抓取开始状态量反馈")
                        robotcontrol.dh_robot_poschange('pos_z', 0.05, 0.02, +0.12) #0.12 1cm

                    elif listrecv_databuf[8] == '03':
                        del listrecv_databuf[:] #清空列表
                        print("任务结束状态量反馈") 

                    elif listrecv_databuf[8] == '04':
                        del listrecv_databuf[:] #清空列表
                        # print("触觉滑动状态量反馈") 

                    elif listrecv_databuf[8] == 'f4':
                        del listrecv_databuf[:] #清空列表
                        print("Confirm the Sample Trial")  

                    elif listrecv_databuf[8] == 'f5':
                        del listrecv_databuf[:] #清空列表
                        print("Confirm Water Botton Trial")                                            

    else:
         del listrecv_databuf[:] #清空列表    



