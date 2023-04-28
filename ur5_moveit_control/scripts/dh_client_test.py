import rospy

from moveit_control.dh_hand_client import *

if __name__ == '__main__':
    try:
        rospy.init_node('actuate_hand_client')
        dh_client = DHClient()

        # result = dh.go_position()
        result = dh.hand_close(50)
        # result = dh.hand_open()
        print("Result:", result.opration_done)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)