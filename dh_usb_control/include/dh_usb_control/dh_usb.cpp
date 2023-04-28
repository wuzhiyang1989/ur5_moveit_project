#include"dh_usb_control/dh_usb.h"

dhUsbControl::dhUsbControl(ros::NodeHandle* nh):nh_(*nh),
                        init_hand_{0x01, 0x06, 0x01, 0x00, 0x00, 0x01, 0x49, 0xf6},
                        force_cmd_{0x01, 0x06, 0x01, 0x01, 0x00},
                        position_cmd_{0x01, 0x06, 0x01, 0x03}
{
    // ROS_INFO("EEE");
    init_serial();

    // init dh hand
    ser_.write(init_hand_, 8);
}

dhUsbControl::~dhUsbControl()
{
}

int dhUsbControl::init_serial(){
    try {
        ser_.setPort("/dev/ts_hand");                   // 修改USB端口
        ser_.setBaudrate(115200);                       // 修改USB波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(to);
        ser_.open();
        ser_.flush();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ser_.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else {
        return -1;
    }
    return 0;
}

bool dhUsbControl::dh_hand_controller(char cmd, int data){
    // std::cout << cmd << data << std::endl;
    if(cmd == 'f' || cmd == 'F'){
        // std::cout << cmd << data << std::endl;
        // 力控指令
        force_cmd_[5] = (unsigned char)(data);                // 取力量数据
        crc_ = get_crc16(force_cmd_, 6);
        force_cmd_[6] = crc_ & 0xff;                          // 取 crc 校验码 低 8 位
        force_cmd_[7] = (crc_ >> 8) & 0xff;                   // 取 crc 校验码 高 8 位
        try{
            ser_.write(force_cmd_, 8);    
        }
        catch(...){ 
            std::cout << "catch(...)" << std::endl;
            // res.success = false;
            return false;
        }
    }
    else if(cmd == 'p' || cmd == 'P'){
        // std::cout << cmd << data << std::endl;
        // 位控指令
        position_cmd_[4] = (unsigned char)((data >> 8) & 0xff);   // 取位置数据高 8 位
        position_cmd_[5] = (unsigned char)(data & 0xff);          // 取位置指令低 8 位
        crc_ = get_crc16(position_cmd_, 6);
        position_cmd_[6] = crc_ & 0xff;                        // 取 crc 校验码 低 8 位
        position_cmd_[7] = (crc_ >> 8) & 0xff;                 // 取 crc 校验码 高 8 位
        
        try{
            ROS_INFO("aaa");
            ser_.write(position_cmd_, 8);    
            ROS_INFO("aaa");
        }
        catch(...){ 
            std::cout << "catch(...)" << std::endl;
            // res.success = false;
            return false;
        }
    }
    // res.success = true;
    return true;
}


uint16_t dhUsbControl::get_crc16(const uint8_t *Ptr, int len){
    uint16_t tmp, CRC16;

    CRC16=0xffff;
    for (int i=0;i<len;i++) {  
        CRC16 = *Ptr ^ CRC16;          
        // std::cout << "*Ptr = " << hex << (unsigned int)(*ptr) << std::endl; 
        for (int j=0;j<8;j++) {
            tmp = CRC16 & 0x0001;
            CRC16 = CRC16 >> 1;
            if (tmp)
                CRC16=CRC16 ^ 0xa001;    
        }
        *Ptr++;
    } 
    return(CRC16);
}