import rospy
import actionlib

from dh_hand_driver.msg    import *
from dh_hand_driver.srv    import *

class DHClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)
        self.client.wait_for_server()

    def go_position(self, MotorID, force, position):
        goal = ActuateHandGoal()
        goal.MotorID  = MotorID
        goal.force    = force
        goal.position = position

        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()


    def hand_close(self, force):
        res = self.go_position(1,int(force),0)
        return res


    def hand_open(self):
        res = self.go_position(1,50,95)
        return res