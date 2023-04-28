#!/usr/bin/env python

from detection.detection import *

if __name__ == "__main__":
    #********************************************************************
    # 获取物品位置流程：
    #   1.获取图像
    #   2.object 检测
    #   3.获取相机坐标
    #   4.坐标转换
    #********************************************************************
    demo = InteractiveInterface()
    image_name = "/home/ts/Pictures/ts_ws/detect_crop.png"
    
    # 抓取流程
    # 获取图像
    demo.image_client()
    # object 检测
    resp = demo.object_client(image_name)
    print(resp)
    # 获取相机坐标
    point = demo.get_point_client(resp.obj[0].colu, resp.obj[0].row)
    print(point)
    # 坐标转换
    translated_point = demo.translate_client(point.position.x, point.position.y, point.position.z)
    print(translated_point)
    # 按照坐标转换结果完成抓取任务


    #********************************************************************
    # 获取放置目标位置流程：
    #   1.获取图像
    #   2.target 检测
    #   3.获取相机坐标
    #   4.坐标转换
    #********************************************************************
    # 放置流程
    # 获取图像
    demo.image_client()
    # object 检测
    resp = demo.target_client(image_name)
    print(resp)
    # 获取相机坐标
    point = demo.get_point_client(resp.obj.colu, resp.obj.row)
    print(point)
    # 坐标转换
    translated_point = demo.translate_client(point.position.x, point.position.y, point.position.z)
    print(translated_point)
    # 按照坐标转换结果完成放置任务