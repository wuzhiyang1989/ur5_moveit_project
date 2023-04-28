#!/home/wy/anaconda3/envs/pytorch/bin/python3
import rospy
import threading
import torch

from geometry_msgs.msg import Pose
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
'''
Aruco:
    620:position:
        x: -0.4634203016757965
        y: 0.061129745095968246
        z: 0.8397260904312134

    621:position:
        x: -0.2647506296634674
        y: 0.06703901290893555
        z: 0.8501524329185486

    622:position:
        x: -0.05064527690410614
        y: 0.07060480862855911
        z: 0.8453758358955383
    623:position:
        x: 0.17425113916397095
        y: 0.0696168914437294
        z: 0.8595675826072693

    624:position:
        x: 0.19032274186611176
        y: -0.09263095259666443
        z: 0.8785332441329956

    625:position:
        x: 0.18553155660629272
        y: 0.227126806974411
        z: 0.8315686583518982

'''
class transfer(object):
    """docstring for transfer"""
    def __init__(self):
        self.sort = [[-0.463, 0.061],
                     [-0.264, 0.067],
                     [-0.050, 0.070],
                     [+0.174, 0.069]]
        self.time = [0 for _ in range(10)]
        self.order = [[0.190, -0.092],
                      [0.174, +0.069],
                      [0.185, +0.227]]
        self.state = torch.zeros((10,12), dtype = int)
 

        rospy.init_node('transfer_state', anonymous=True)

        rospy.Subscriber("/aruco_simple/pose_601", Pose, self.callback_obj01)
        rospy.Subscriber("/aruco_simple/pose_602", Pose, self.callback_obj02)
        rospy.Subscriber("/aruco_simple/pose_603", Pose, self.callback_obj03)
        rospy.Subscriber("/aruco_simple/pose_604", Pose, self.callback_obj04)
        rospy.Subscriber("/aruco_simple/pose_605", Pose, self.callback_obj05)
        rospy.Subscriber("/aruco_simple/pose_606", Pose, self.callback_obj06)
        rospy.Subscriber("/aruco_simple/pose_607", Pose, self.callback_obj07)
        rospy.Subscriber("/aruco_simple/pose_608", Pose, self.callback_obj08)
        rospy.Subscriber("/aruco_simple/pose_609", Pose, self.callback_obj09)
        rospy.Subscriber("/aruco_simple/pose_610", Pose, self.callback_obj10)
        rospy.Subscriber("/aruco_simple/pose_611", Pose, self.callback_obj11)
        rospy.Subscriber("/aruco_simple/pose_612", Pose, self.callback_obj12)

        self.add_thread = threading.Thread(target=self.spin_thread)
        self.add_thread.start()
        self.time_thread = threading.Thread(target=self.spin_time_thread)
        self.time_thread.start()

    def detection(self, position):
        if position.x > self.sort[0][0] and position.x < self.sort[1][0]:
            if position.y > self.sort[0][1]:
                return 14         # sort 3
            else:
                return 17         # sort 6
        if position.x > self.sort[1][0] and position.x < self.sort[2][0]:
            if position.y > self.sort[1][1]:
                return 13         # sort 2
            else:
                return 16         # sort 5
        if position.x > self.sort[2][0] and position.x < self.sort[3][0]:
            if position.y > self.sort[2][1]:
                return 12         # sort 1
            else:
                return 15         # sort 4
        if position.x > self.order[0][0] and position.y < self.order[0][1]:
            return 20             # trash
        if position.x > self.order[2][0] and position.y > self.order[2][1]:
            return 21             # order 1
        if position.y > self.order[1][1] and position.y < self.order[2][1] and position.x > self.order[1][0]:
            return 18             # order 2
        if position.y > self.order[0][1] and position.y < self.order[1][1] and position.x > self.order[1][0]:
            return 19             # order 3
        return -1

    def callback_obj01(self, data): 
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1: 
            self.state[:, 0] = 0
            self.state[index-12][0] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj02(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 1] = 0
            self.state[index-12][1] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj03(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 2] = 0
            self.state[index-12][2] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj04(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 3] = 0
            self.state[index-12][3] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj05(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 4] = 0
            self.state[index-12][4] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj06(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 5] = 0
            self.state[index-12][5] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj07(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 6] = 0
            self.state[index-12][6] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj08(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 7] = 0
            self.state[index-12][7] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj09(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 8] = 0
            self.state[index-12][8] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj10(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 9] = 0
            self.state[index-12][9] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj11(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 10] = 0
            self.state[index-12][10] = 1
            self.time[index -12] = rospy.get_time()

    def callback_obj12(self, data):
        '''
        deal with aruco marker pose
        '''
        index = self.detection(data.position)
        if not index == -1:
            self.state[:, 11] = 0
            self.state[index-12][11] = 1
            self.time[index -12] = rospy.get_time()


    def pub_state(self):
        pub   = rospy.Publisher('/transfer_state', Int8MultiArray, queue_size = 1)
        # ros msg 
        msg  = Int8MultiArray()
        dim_msg1 = MultiArrayDimension()
        dim_msg2 = MultiArrayDimension()
        dim_msg1.size   = 10
        dim_msg1.stride = 120
        dim_msg1.label  = "area"
        msg.layout.dim.append(dim_msg1)
        dim_msg2.size   = 12
        dim_msg2.stride = 12
        dim_msg2.label  = "product"
        msg.layout.dim.append(dim_msg2)
        # msg.layout.data_offset = 20

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t_now = rospy.get_time()
            print(rospy.get_time())
            msg.data = self.state.reshape(120).tolist()

            print(msg)
            pub.publish(msg)
            rate.sleep()


    def spin_thread(self):
        rospy.spin()

    def spin_time_thread(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            t_now = rospy.get_time()
            index = 0
            for t in self.time:
                if t_now - t > 0.5:
                    for i in range(12):
                        self.state[index][i] = 0
                index = index + 1
            rate.sleep()


if __name__ == "__main__":
    T = transfer()
    T.pub_state()