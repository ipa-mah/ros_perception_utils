#include "ros_perception_utils/rosbag_interface.hpp"

BagInterface::BagInterface(const ros::NodeHandle& nh, const std::string& rgb_topic, const std::string& aligned_depth_topic, const std::string& cam_info_topic) : nh_(nh)
{
	rgb_pub_ = nh_.advertise<sensor_msgs::Image>(rgb_topic, 1);
	depth_pub_ = nh_.advertise<sensor_msgs::Image>(aligned_depth_topic, 1);
	rgb_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);
	status_pub_ = nh_.advertise<std_msgs::Bool>("bagfile_status", 1);
}

BagInterface::~BagInterface()
{
}
void BagInterface::publishRGBDInfoTopics(const std::string& bag_file_path, const std::string& rgb_topic, const std::string& aligned_depth_topic, const std::string& cam_info_topic, int frame_rate)
{
	ros::Rate rate(frame_rate);
	rosbag::Bag bag;
	std::string bag_file = bag_file_path + "/rec.bag";

	bag.open(bag_file, rosbag::bagmode::Read);

	std_msgs::Bool msgs_status;
	msgs_status.data = true;
	while (ros::ok())
	{
		std::vector<std::string> topics;
		topics.push_back(rgb_topic);
		topics.push_back(aligned_depth_topic);
		topics.push_back(cam_info_topic);
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		int num_messages = view.size();

		BOOST_FOREACH (rosbag::MessageInstance const m, view)
		{
			if (m.getTopic() == rgb_topic)
			{
				sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>();
				if (rgb_image_ptr != NULL) rgb_pub_.publish(*rgb_image_ptr);
			}
			if (m.getTopic() == aligned_depth_topic)
			{
				sensor_msgs::Image::ConstPtr aligned_depth_ptr = m.instantiate<sensor_msgs::Image>();
				if (aligned_depth_ptr != NULL) depth_pub_.publish(*aligned_depth_ptr);
			}
			if (m.getTopic() == cam_info_topic)
			{
				sensor_msgs::CameraInfo::ConstPtr rgb_info_ptr = m.instantiate<sensor_msgs::CameraInfo>();
				if (rgb_info_ptr != NULL) rgb_info_pub_.publish(*rgb_info_ptr);
			}
			status_pub_.publish(msgs_status);
			rate.sleep();
		}
		msgs_status.data = false;
		status_pub_.publish(msgs_status);
		ROS_WARN_NAMED("eval", "Finished processing of Bagfile");
		bag.close();
		break;
	}
}
