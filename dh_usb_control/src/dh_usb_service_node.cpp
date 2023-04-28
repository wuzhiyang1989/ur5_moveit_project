#include "ros/ros.h"
#include "dh_usb_control/DHCmd.h"
#include "dh_usb_control/dh_usb.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "dh_usb_control");
    ros::NodeHandle n;
    dhUsbControl dh(&n);

    char cmd_ = 'a';
    int data_ = 0;

    boost::shared_ptr<dh_usb_control::DHCmd const> msg;
    
    while(true){
        msg = ros::topic::waitForMessage<dh_usb_control::DHCmd>("ts_hand_cmd");
        if(cmd_ == msg->cmd && data_ == msg->data)
            continue;
        else{
            cmd_ = msg -> cmd;
            data_ = msg -> data;
            dh.dh_hand_controller(cmd_, data_);
        }
    }
    return 0;
}