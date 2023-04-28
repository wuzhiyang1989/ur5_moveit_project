#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray, MultiArrayDimension

def talker():
    rospy.init_node('franka_command_test', anonymous=True)
    pub   = rospy.Publisher('franka_cmd', Int8MultiArray, queue_size=1)

    # ros msg 
    msg  = Int8MultiArray()
    dim_msg  = MultiArrayDimension()
    dim_msg.size   = 3
    dim_msg.stride = 3
    dim_msg.label  = "cmd"
    msg.layout.dim.append(dim_msg)

    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        data = np.random.randint(0, 20, size = (3)).tolist()
        msg.data = data
        print(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass