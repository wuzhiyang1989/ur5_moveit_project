#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hm_cooperation/GetImg.h"
#include "hm_cooperation/image/hm_image.h"


using namespace std;

void cmdCallback(const std_msgs::String::ConstPtr& msg, HMImage *hm_ptr) {

    string cmd = msg->data;
    ROS_INFO("Recieved command: %s", cmd);
    if(cmd != "False"){
        hm_ptr->SaveImage();
        ROS_INFO("Color image save succeed.");
        hm_ptr->SaveState();
    }
}

bool save_image(hm_cooperation::GetImg::Request &req, hm_cooperation::GetImg::Response &res, HMImage *hm_ptr){
    res.image = "att";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImageUnit");

    ros::NodeHandle n;

    HMImage hm_image(&n);
    // while(1) {
    //     ros::Subscriber sub = ros::topic::waitForMessage<std_msgs::String>("image_cmd", 1000, boost::bind(&cmdCallback, _1, &hm_image));

    // }
    ros::Subscriber sub = n.subscribe<std_msgs::String>("image_cmd", 1000, boost::bind(&cmdCallback, _1, &hm_image));
    
    // ros::ServiceServer service = n.advertiseService("get_image", boost::bind(&save_image, _1, _2, &hm_image));
    // ros::ServiceServer service = n.advertiseService<hm_cooperation::GetImg>("get_image", save_image);

    ros::spin();

    return 0;
}