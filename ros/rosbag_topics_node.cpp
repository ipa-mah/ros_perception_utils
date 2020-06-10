#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <ros_perception_utils/rosbag_interface.hpp>



int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_topic_node");

    const std::string topic_file = ros::package::getPath("ipa_scanstation_to_rgbd_dataset")+ "/config.yaml";
    ros::NodeHandle node;
    cv::FileStorage fs(topic_file,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr<<"no topic file detected ..."<<std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::string rgb_topic, depth_topic,rgb_info_topic;
    fs["rgb_topic"]>>rgb_topic;
    std::cout<<"rgb_topic: "<<rgb_topic<<std::endl;
    fs["depth_topic"]>>depth_topic;
    std::cout<<"depth_topic: "<<depth_topic<<std::endl;
    fs["rgb_info_topic"]>>rgb_info_topic;
    std::cout<<"rgb_info_topic: "<<rgb_info_topic<<std::endl;
    std::string bag_file_path;
    fs["bag_file_path"]>>bag_file_path;
    std::cout<<"bag_file_path: "<<bag_file_path<<std::endl;
    int frame_rate;
    fs["rate"]>>frame_rate;
    std::cout<<"frame_rate: "<<frame_rate<<std::endl;

    BagInterface bag(node,rgb_topic,depth_topic,rgb_info_topic);
    bag.publishRGBDInfoTopics(bag_file_path,rgb_topic,depth_topic,rgb_info_topic,frame_rate);
    return 0;
}
