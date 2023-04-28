#include <iostream>

#include "ros/ros.h"
#include "serial/serial.h"
class dhUsbControl
{
private:
    serial::Serial ser_;
    ros::NodeHandle nh_;
    
    uint16_t crc_;
    uint8_t init_hand_[8];
    uint8_t force_cmd_[8];
    uint8_t position_cmd_[8];
public:
    dhUsbControl(ros::NodeHandle* nh);
    ~dhUsbControl();

    int init_serial();
    bool dh_hand_controller(char cmd, int data);
    uint16_t get_crc16(const uint8_t *Ptr, int len);
};

