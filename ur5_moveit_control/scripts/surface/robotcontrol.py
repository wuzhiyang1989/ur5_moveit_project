#! /usr/bin/env python
# coding=utf-8
import time
import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Process, Queue
import os
from math import pi
import downcontrol  

import numpy

import urx
import sys


def dh_robot_init():
    global dh_robot

    dh_robot = urx.Robot("192.168.56.2")

    pose = dh_robot.get_pos()
    print("getl")
    print(pose)
    """
    net_count_try = 3
    while net_count_try:
        dh_robot = urx.Robot("192.168.56.2")
        net_count_try -= 1
    """
def dh_robot_poschange(dirpara, accvalue, speedvalue, posvalue):
  
    current_pos = dh_robot.getl()

    if dirpara == 'pos_x':
        current_pos[0] = (current_pos[0] + posvalue)
    elif dirpara == 'pos_y':
        current_pos[1] = (current_pos[1] + posvalue)
    elif dirpara == 'pos_z':
        current_pos[2] = (current_pos[2] + posvalue)    
  
    dh_robot.movel(current_pos, accvalue, speedvalue, wait=False)

    current_lastpos = dh_robot.get_pos()
    if current_lastpos[2] >= 0.1:
        if current_lastpos[2] <= 0.232:
            downcontrol.dh_uart_send('ArrivePos') 
            print("get the postion")     

def dh_robot_topchangepos(accvalue, speedvalue,gesturepara):
    """最高点改变姿态"""
    
    if gesturepara == 1: #竖直
        joint_radian = (87.445*pi/180, 13.33*pi/180, -76.31*pi/180, 1.87*pi/180, -92.76*pi/180, -93.508*pi/180)
        # logger.info("move joint to {0}".format(joint_radian))
        #dh_robot.move_joint(joint_radian)  
    elif gesturepara == 2: #水平
        joint_radian = (87.445*pi/180, 13.33*pi/180, -76.31*pi/180, 1.87*pi/180, -174.76*pi/180, -93.508*pi/180)
        # logger.info("move joint to {0}".format(joint_radian))
        #dh_robot.move_joint(joint_radian)  
    elif gesturepara == 3: #抓水平原点
        joint_radian = (-38.88*pi/180, -124.74*pi/180, -93.42*pi/180, -143.50*pi/180, -37.13*pi/180, -0*pi/180)
        dh_robot.movej(joint_radian, accvalue, speedvalue) 