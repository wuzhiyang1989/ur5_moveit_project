#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hm_cooperation/franka/hm_franka.h"
// #include "hm_cooperation/franka/hm_gripper.h"


using namespace std;

void cmdCallback(const std_msgs::String::ConstPtr& msg, HMFranka *hm_ptr) {

    string cmd = msg->data;
    ROS_INFO("Recieved command: %s", cmd.c_str());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FrankaUnit");

    ros::NodeHandle n;

    HMFranka hm_franka(&n);
    // HMGripper hm_gripper(&n);
    
    ros::Subscriber sub = n.subscribe<std_msgs::String>("franka_cmd", 1000, boost::bind(&cmdCallback, _1, &hm_franka));

    ros::spin();

    return 0;
}