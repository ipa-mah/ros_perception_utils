#include "ros_perception_utils/ros_rgbd_camera.hpp"
#include <ros/console.h>

// constructor to subcribe RGBD topics
RosRGBDCamera::RosRGBDCamera(const ros::NodeHandle& handle, const std::string& rgb_topic, const std::string& depth_topic, const std::string& rgb_info_topic)
  : nh_(handle),
	it_(handle),
	rgb_sub_(nh_, rgb_topic, 1),
	depth_sub_(nh_, depth_topic, 1),
	rgb_info_sub_(nh_, rgb_info_topic, 1),
	sync_subs(Policy_sync_subs(9), rgb_sub_, depth_sub_, rgb_info_sub_),
	subscribed_(false),
	bag_status_(false)

{
	sync_subs.registerCallback(boost::bind(&RosRGBDCamera::rgbdCallBack, this, _1, _2, _3));
	bag_status_sub_ = nh_.subscribe("/bagfile_status", 1, &RosRGBDCamera::bagCallBack, this);
}

RosRGBDCamera::~RosRGBDCamera()
{
}
void RosRGBDCamera::rgbdCallBack(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::ImageConstPtr& image_depth_msg, const sensor_msgs::CameraInfoConstPtr rgb_info_msg)
{
	last_rgb_ = image_color_msg;
	last_depth_ = image_depth_msg;
	last_rgb_info_ = rgb_info_msg;  // for further use

	subscribed_ = true;
}
void RosRGBDCamera::bagCallBack(const std_msgs::BoolConstPtr& bag_status)
{
	bag_status_ = bag_status->data;
}

bool RosRGBDCamera::grab(cv::Mat& rgb, cv::Mat& depth)
{
	if (subscribed_ == false) return false;
	cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvCopy(last_depth_, "");
	if (!cv_depth)
	{
		ROS_ERROR("Failed to convert depth to opencv image");
		return false;
	}
	depth = cv_depth->image.clone();
	;
	cv_bridge::CvImageConstPtr cv_rgb = cv_bridge::toCvCopy(last_rgb_, "bgr8");
	if (!cv_rgb)
	{
		ROS_ERROR("Failed to convert rgb to opencv image");
		return false;
	}
	rgb = cv_rgb->image.clone();
	;
	subscribed_ = false;
	return true;
}
