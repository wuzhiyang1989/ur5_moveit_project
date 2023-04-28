#ifndef  HM_FRANKA_H
#define  HM_FRANKA_H

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/duration.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include "hm_cooperation/PoseGoal.h"
#include "hm_cooperation/JointSpaceGoal.h"
#include "hm_cooperation/ManipulatorInfo.h"
#include "hm_cooperation/ManipulatorState.h"
#include "hm_cooperation/franka/hm_gripper.h"

using namespace std;

static double detect_point_[7] = {-1.570792, 0, 0, -1.571549, 0, 1.570432, 0.785676};
class HMFranka {

public:

	HMFranka(ros::NodeHandle* nh);
	~HMFranka();

	void EchoManipulatorInfo();
	void EchoManipulatorState();
	// void SendPoseGoal();
	void SendJointGoal();

	void OnMission(int mission_index);

private:

	HMGripper hm_gripper_;
	ros::ServiceClient info_client_;
	ros::ServiceClient pose_client_;
	ros::ServiceClient joint_client_;
	ros::ServiceClient state_client_;

	hm_cooperation::PoseGoal         pose_srv_;
	hm_cooperation::JointSpaceGoal   joint_srv_;
	hm_cooperation::ManipulatorInfo  info_srv_;
	hm_cooperation::ManipulatorState state_srv_;
};

#endif