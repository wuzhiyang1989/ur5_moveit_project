#!/usr/bin/env python

import rospy
from ocrtoc_common.manipulator_interface import ManipulatorInterface

if __name__ == '__main__':
    rospy.init_node('manipulator_interface_node')
    m_i = ManipulatorInterface('panda_arm')
    # m_i = ManipulatorInterface('manipulator')
    m_i.print_basic_info()
    resp = m_i.get_manipulator_state_handler()
    print(resp.current_pose)
    print(resp.joint_position_list)

    print("Try to change max_velocity_scaling_factor")
    m_i.set_max_velocity_scaling(1)
    print("Try to change max_acceleration_scaling_factor")
    m_i.set_max_acceleration_scaling(1)

    rospy.spin()
