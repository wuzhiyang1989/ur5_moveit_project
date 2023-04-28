#!/usr/bin/env python3
import time
import rospy

from franka_control import MoveITControl
from hm_data import *

def main():
    rospy.init_node('franka_correct_product', anonymous=True)
    product_list = [1, 2, 4, 5, 7, 9, 11]  # executble product index list
    # product_list = [1, 2, 4, 9]
    # product_list = [1]
    # product 0, 3, 6, 8, 10 can't ececute
    
    # go init joint
    correction = MoveITControl()
    correction.set_scaling_factor(0.1)

    correction.panda_arm.go(init_joint, wait=True)
    print("Arrived init joint.")
    for i in product_list:
        # key = input("Input cmd << ")
        # if key != 'q' and key != 'Q':
        # t1 = time.time()
        # print(f"t1 = {t1}")
        correction.panda_arm.go(prepare_joint[i], wait=True)
        # t2 = time.time()
        # print(f"t2 = {t2}")
        # key = input("Input cmd << ")
        # if key != 'q' and key != 'Q':
        correction.set_scaling_factor(0.2)
        correction.panda_arm.go(object_joint[i], wait=True)      # max： 0.2
        key = input("Input cmd << ")
        if key != 'q' and key != 'Q':
            correction.panda_arm.go(prepare_joint[i], wait=True)
        correction.set_scaling_factor(0.5)
        # key = input("Input cmd << ")
        # if key != 'q' and key != 'Q':
        correction.panda_arm.go(init_joint, wait=True)
        # t2 = time.time()
        # print(f"t2 = {t2}")

        # print(f"t2 - t1 = {t2 - t1}")
        # print(f"t3 - t2 = {t3 - t2}")
        if i == product_list[-1]:
            print("Correcting finished.")
        else:
            print("Next object...")
    pass

if __name__ == '__main__':
    main()

        #https://github.com/ros-planning/moveit/issues/86 