#! /usr/bin/env python
# coding=UTF-8
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf

def tf_publish_init(parent_name, child_name):
    ts = TransformStamped()
    ts.header.frame_id = parent_name  #kinect
    ts.child_frame_id = child_name
    return ts


if __name__=="__main__":
    rospy.init_node("dynamic_pub_tf") # 初始化节点

    ts_1 = tf_publish_init("world", "kinect")
    ts_2 = tf_publish_init("kinect", "bottle")
    pub  = tf2_ros.TransformBroadcaster()

    ts_1.transform.translation.x = 0
    ts_1.transform.translation.y = 0
    ts_1.transform.translation.z = 0

    ts_1.transform.rotation.x =  0
    ts_1.transform.rotation.y =  0
    ts_1.transform.rotation.z =  0
    ts_1.transform.rotation.w =  1

    ts_2.transform.translation.x = 1
    ts_2.transform.translation.y = 2
    ts_2.transform.translation.z = 3

    ts_2.transform.rotation.x =  0.7190
    ts_2.transform.rotation.y = -0.6943
    ts_2.transform.rotation.z =  0.0194
    ts_2.transform.rotation.w =  0.0206

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # data = rospy.wait_for_message("/object_position", Point)

        ts_1.header.stamp = rospy.Time.now()
        # ts_2.header.stamp = rospy.Time.now()

        # ts.transform.translation.x = 1
        # ts.transform.translation.y = 2
        # ts.transform.translation.z = 3

        # ts.transform.rotation.x =  0.7190
        # ts.transform.rotation.y = -0.6943
        # ts.transform.rotation.z =  0.0194
        # ts.transform.rotation.w =  0.0206

        pub.sendTransform(ts_1)
        # pub.sendTransform(ts_2)
        print("publish successed {}".format(rospy.Time.now().secs))
        
        rate.sleep()