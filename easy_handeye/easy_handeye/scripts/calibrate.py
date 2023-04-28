#!/home/wy/anaconda3/envs/rosenv/bin python2.7
import rospy

from easy_handeye.handeye_server import HandeyeServer


def main():
    rospy.init_node('easy_handeye')
    while rospy.get_time() == 0.0:
        pass

    cw = HandeyeServer()

    rospy.spin()


if __name__ == '__main__':
    main()




# print("#!/usr/bin/env python")


