#include "hm_cooperation/franka/hm_franka.h"

HMFranka::HMFranka(ros::NodeHandle* nh):hm_gripper_(nh){
	
	// pose_client_  = nh->serviceClient<hm_cooperation::PoseGoal>("send_pose_goal");
	// info_client_  = nh->serviceClient<hm_cooperation::ManipulatorInfo>("get_manipulator_info");
	// joint_client_ = nh->serviceClient<hm_cooperation::JointSpaceGoal>("send_joint_space_goal");
	// state_client_ = nh->serviceClient<hm_cooperation::ManipulatorState>("get_manipulator_state");

	// pose_client_ .waitForExistence();
	// info_client_ .waitForExistence();
	// joint_client_.waitForExistence();
	// state_client_.waitForExistence();

}

HMFranka::~HMFranka() {

}

void HMFranka::EchoManipulatorInfo() {

	if (info_client_.call(info_srv_)) {
		// ROS_INFO("Group names: %s", info_srv_.response.group_names);
		ROS_INFO("End effector link: %s", info_srv_.response.end_effector_link.c_str());
		ROS_INFO("Planning frame: %s", info_srv_.response.planning_frame.c_str());
	}
}

void HMFranka::EchoManipulatorState() {

	// if (state_client_.call(state_srv_)) {
		// 暂时不做输出
	// }
}

void HMFranka::SendJointGoal() {

	vector<double> joint(detect_point_, detect_point_ + sizeof(detect_point_) / sizeof(detect_point_[0]));
	joint_srv_.request.joint_goal = joint;

	joint_client_.call(joint_srv_);
	ROS_INFO("Move state: %d", joint_srv_.response.successed);
	ROS_INFO("Move state: %s", joint_srv_.response.text.c_str());
}

void HMFranka::OnMission(int mission_index) {
/*************************************************************************************
 * Task list:
 * 		obtian: task index --> box index, box size, position, label orientation
 * 
 * Grab the box: 
 * 		get task list --> get task index --> move robot up to the box --> down -->
 * 		--> gripper close -->move robot to detect point --> end
 * 
 * Detect label:
 * 		orientate == true:  Wrist camera is better.(eg. realsense)
 * 			take box to translate point --> put down --> turn robot's orientation -->
 * 			--> read aruco result --> robot move up --> read aruco result  -->caculate
 * 		orientate == false:
 * 			take box to check point --> left 90 --> check aruco --> right 180 --> check aruco -->
 * 			--> turn box bottom  --> check aruco --> take box to translate point -->
 * 			-->turn gripper 90 --> move to detect point --> turn 180 
 * 			if check --> orientation = ture  
 * 				else --> detect expectation = max
 * Sort:
 * 		put down box --> turn robot's orientation --> get box --> move to sort area.
*************************************************************************************/
	
}
/**************************************************************************************
 * sendJointGoal example:例程，可以不设定send代码
 * 毕竟send仅需要赋值，修改，send三个步骤
 * 改为抓取-->检测-->放置3个函数进行封装
 * 
 * 完成检测node代码，以monitor node为核心进行通信和控制
**************************************************************************************/