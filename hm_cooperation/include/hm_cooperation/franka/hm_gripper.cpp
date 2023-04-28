#include "hm_cooperation/franka/hm_gripper.h"

HMGripper::HMGripper(ros::NodeHandle* nh):
gripper_topic_("/franka_gripper/gripper_action"),
gripper_client_(gripper_topic_, true) {
	
	goal_.command.position   = 0;
	goal_.command.max_effort = 10;
}

HMGripper::~HMGripper() {
    
}

void HMGripper::GoToPosition(float position, float effort) {

// Move the gripper to position
// Args:
//     position(float: m): the target distance between the two fingers
//     max_effort(float) : max effort
//     wait_time(float)  : time to wait after sending the command.

	gripper_client_.waitForServer();
	goal_.command.position   = position / 2.0;
	goal_.command.max_effort = effort;
	gripper_client_.sendGoal(goal_);

}

void HMGripper::Open() {

	GoToPosition(0.078, 10);

}

void HMGripper::Close() {

	GoToPosition(0.0,   10);

}
