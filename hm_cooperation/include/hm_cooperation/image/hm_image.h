#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "hm_cooperation/GetImg.h"
#include "sensor_msgs/image_encodings.h"

#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;


class HMImage {
public:
	HMImage(ros::NodeHandle* nh);
	~HMImage();

	// void SaveImage();
	bool SaveImage();
	void SaveState();
	void CropSave(cv::Mat &img);
	// void ColorSubCallback(const sensor_msgs::Image::ConstPtr& color_image);
	// void DepthSubCallback(const sensor_msgs::Image::ConstPtr& depth_image);


private:
	cv::Mat image_;
	string image_cmd_;
	ros::NodeHandle nh_;
	// ros::Subscriber color_sub_;
	// ros::Subscriber depth_sub_;
	ros::Publisher save_state_;
    cv_bridge::CvImageConstPtr cv_ptrRGB_;
    boost::shared_ptr<sensor_msgs::Image const> shared_image_;

};