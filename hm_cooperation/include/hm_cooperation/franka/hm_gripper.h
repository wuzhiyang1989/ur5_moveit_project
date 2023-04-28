#ifndef  HM_GRIPPER_H
#define  HM_GRIPPER_H

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/duration.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h> 
#include <actionlib/client/simple_action_client.h>

using namespace std;

class HMGripper{
public:
	HMGripper(ros::NodeHandle* nh);
	~HMGripper();

	void Open();
	void Close();
	void GoToPosition(float position, float effort = 10.0);

private:
	// ros::NodeHandle nh_;
	string gripper_topic_;

	control_msgs::GripperCommandGoal goal_;
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;

};

#endif