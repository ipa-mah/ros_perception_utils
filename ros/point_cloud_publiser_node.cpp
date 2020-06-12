#include <ros/ros.h>
#include <iostream>
#include <ros_perception_utils/point_cloud_publisher.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_publisher_node");
    ros::NodeHandle node;
    PointCloudPushliser::Ptr pc_publisher = std::make_shared<PointCloudPushliser>(node);
    ros::spin();
    return 0;
}

