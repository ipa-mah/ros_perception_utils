#pragma once
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <tuple>
class RosRGBDCamera
{
  public:
  using Ptr = std::shared_ptr<RosRGBDCamera>;
  using ConstPtr = std::shared_ptr<const RosRGBDCamera>;
	RosRGBDCamera(const ros::NodeHandle& handle, const std::string& rgb_topic, const std::string& depth_topic, const std::string& rgb_info_topic);
	virtual ~RosRGBDCamera();

	void rgbdCallBack(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::ImageConstPtr& image_depth_msg, const sensor_msgs::CameraInfoConstPtr rgb_info_msg);
	void bagCallBack(const std_msgs::BoolConstPtr& bag_status);
	bool grab(cv::Mat& rgb, cv::Mat& depth);
	bool bag_status_;
	bool subscribed_;

  protected:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_info_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> Policy_sync_subs;
	message_filters::Synchronizer<Policy_sync_subs> sync_subs;

	sensor_msgs::ImageConstPtr last_depth_;
	sensor_msgs::ImageConstPtr last_rgb_;
	sensor_msgs::CameraInfoConstPtr last_rgb_info_;
	ros::Subscriber bag_status_sub_;

	Eigen::Matrix3d rgb_intrins_;
  int color_width_;
  int color_height_;

  std::tuple<int, int , Eigen::Matrix3d> getColorIntrinsInfo();
};

inline std::tuple<int, int , Eigen::Matrix3d> RosRGBDCamera::getColorIntrinsInfo()
{
  return std::make_tuple(color_width_,color_height_,rgb_intrins_);
}

