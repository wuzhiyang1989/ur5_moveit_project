#include "hm_cooperation/image/hm_image.h"

HMImage::HMImage(ros::NodeHandle* nh):nh_(*nh) {
    // color_sub_ = nh_.subscribe<sensor_msgs::Image>("kinect2/hd/color_image", 1, &HMImage::ColorSubCallback, this);
    // depth_sub_ = nh_.subscribe<sensor_msgs::Image>("kinect2/hd/depth_image", 1, &HMImage::DepthSubCallback, this);
    save_state_ = nh_.advertise<std_msgs::String>("save_state", 1);

}

HMImage::~HMImage() {

}


bool HMImage::SaveImage(){

    shared_image_ = ros::topic::waitForMessage<sensor_msgs::Image>("kinect2/hd/image_color", nh_);

    if(shared_image_ != NULL){
        cv_ptrRGB_ = cv_bridge::toCvCopy(*shared_image_);
    }

    image_ = cv_ptrRGB_->image;

    if(image_.channels()==3) {
        ROS_INFO("image_.channels() = %d", image_.channels());
        // cv::cvtColor(image_,image_,CV_BGR2RGB);
    }

    cv::imwrite("/home/ts/Pictures/ts_ws/detect.png",image_);
	// ROS_INFO("Save succeed.");
    CropSave(image_);
    return true;
}

// void HMImage::CropSave(cv::Mat &img, cv::Mat &crop_img, std::vector<int> &area){
// (160.000000 274.000000 1486.000000 901.000000)
void HMImage::CropSave(cv::Mat &img){
    //int area[] = {320, 255, 1280, 760};
    int area[] = {160, 400, 1485, 900};     // (160  275) 截取图片左上角，（1485 900）长宽
    // int area[] = {1500, 1500, 600, 600}; 
    
    int crop_x1 = std::max(0, area[0]);
    int crop_y1 = std::max(0, area[1]);
    // int crop_x2 = std::min(img.cols -1, area[0] + area[2] - 1);
    // int crop_y2 = std::min(img.rows - 1, area[1] + area[3] - 1);
    int crop_x2 = std::min(img.cols -1, area[2] - 1);
    int crop_y2 = std::min(img.rows - 1, area[3] - 1);

    cv::Mat crop_img = img(cv::Range(crop_y1, crop_y2+1), cv::Range(crop_x1, crop_x2 + 1));

    cv::imwrite("/home/ts/Pictures/ts_ws/detect_crop.png", crop_img);
    ROS_INFO("Save succeed.");
}

void HMImage::SaveState(){
    std_msgs::String msg;
    msg.data = "Succeed";
    ROS_INFO("Save succeed!");
    int count = 0;
    ros::Rate loop_rate(10);

    while ((ros::ok() && count < 25)){
        save_state_.publish(msg);
        ros::spinOnce();
        ++count;    
        loop_rate.sleep();
    }

}
