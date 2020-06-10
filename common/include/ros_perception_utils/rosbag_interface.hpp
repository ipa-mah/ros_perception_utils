#ifndef BAG_INTERFACE_HPP
#define BAG_INTERFACE_HPP
// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>

class BagInterface
{
  public:
	BagInterface(const ros::NodeHandle& nh, const std::string& rgb_topic, const std::string& aligned_depth_topic, const std::string& cam_info_topic);
	virtual ~BagInterface();
	void publishRGBDInfoTopics(const std::string& bag_file_path, const std::string& rgb_topic, const std::string& aligned_depth_topic, const std::string& cam_info_topic, int frame_rate);

  private:
	ros::NodeHandle nh_;
	ros::Publisher rgb_pub_;
	ros::Publisher depth_pub_;
	ros::Publisher rgb_info_pub_;
	ros::Publisher status_pub_;

	std::string config_file_;
};
#endif  // BAG_INTERFACE_HPP
