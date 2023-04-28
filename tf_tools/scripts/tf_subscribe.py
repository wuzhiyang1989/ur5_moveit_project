#! /usr/bin/env python3
# coding=UTF-8

import rospy
import time
import tf2_ros
# import tf
from geometry_msgs.msg import PoseStamped,PointStamped, TransformStamped
from tf2_geometry_msgs import PoseStamped,PointStamped

if __name__=="__main__":

    rospy.init_node("static_sub_p")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 定义要转换的输入
    p = PointStamped()
    p.header.stamp = rospy.Time()
    p.header.frame_id = 'kinect'
    p.point.x = 8
    p.point.y = 0
    p.point.z = 0

    # p=PoseStamped()
    # p.header.stamp=rospy.Time()
    # p.header.frame_id='kinect'
    # p.pose.position.x=5
    # p.pose.position.y=0
    # p.pose.position.z=0
    # p.pose.orientation.x=0
    # p.pose.orientation.y=0
    # p.pose.orientation.z=0
    # p.pose.orientation.w=1

 
    res = tf_buffer.transform(p,'world',timeout=rospy.Duration(5))
    print(res)

    # buffer = tf2_ros.Buffer()
    # sub = tf2_ros.TransformListener(buffer)

    # ts = TransformStamped()
    # ts.header.frame_id = "kinect"
    # ts.child_frame_id  = "bottle"
    # ts.header.stamp    = rospy.Time.now()

    # ts.transform.translation.x = 1
    # ts.transform.translation.y = 2
    # ts.transform.translation.z = 3

    # ts.transform.rotation.x =  0.7190
    # ts.transform.rotation.y = -0.6943
    # ts.transform.rotation.z =  0.0194
    # ts.transform.rotation.w =  0.0206

    # pub2  = tf2_ros.TransformBroadcaster()
    # pub2.sendTransform(ts)
    # # 转换逻辑实现，调用tf封装的算法
    # rate = rospy.Rate(30) # 1Hz频率
    # t1 = time.time()
    # print("t1 = ", t1)
    # while not rospy.is_shutdown():
    #     try:
    #         run_time = time.time() - t1
    #         # if run_time < 2:
    #         #     ts.header.stamp    = rospy.Time.now()
    #         #     pub2.sendTransform(ts)

    #         ts.header.stamp    = rospy.Time.now()
    #         pub2.sendTransform(ts)
    #         # 转换实现
    #         ps_out = buffer.lookup_transform("bottle", "world", rospy.Time(0))
    #         # ps_out = buffer.transform()
    #         print("suceed time t =", time.time())
    #         break
    #         # 输出结果
    #         rospy.loginfo("转换后的坐标：(%.2f, %.2f, %.2f), 参考的坐标系：%s",
    #                     ps_out.transform.translation.x, 
    #                     ps_out.transform.translation.y, 
    #                     ps_out.transform.translation.z, 
    #                     ps_out.header.frame_id)

    #     except Exception as e:
    #         et = time.time()
    #         print("Error trime et = ", et)
    #         rospy.loginfo("错误提示：%s", e)

    #     rate.sleep()