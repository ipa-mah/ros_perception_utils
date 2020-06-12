#ifndef POINT_CLOUD_PUBLISHER_HPP
#define POINT_CLOUD_PUBLISHER_HPP
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
class PointCloudPushliser
{
public:
  using Ptr = std::shared_ptr<PointCloudPushliser>;
  using ConstPtr = std::shared_ptr<const PointCloudPushliser>;
  PointCloudPushliser(ros::NodeHandle node);
  virtual ~PointCloudPushliser();
  ///
  /// \brief readParameters function to read parameters from parameter server
  /// \return true if successful
  ///
  bool readParameters();

  ///
  /// \brief initialize function to read organized point cloud
  ///
  void initialize();


  bool readPointCloudFile(const std::string& pc_file, const std::string& pc_frame_id);
protected:
  // ROS node handle
  ros::NodeHandle node_handle_;

  //publish organized point cloud topic
  ros::Publisher pc_publisher_;
  // topic for publishing rgb image
  image_transport::Publisher rgb_image_publisher_;
  // topic for publishing depth image
  image_transport::Publisher depth_image_publisher_;
  //Used to subscribe and publish images.
  image_transport::ImageTransport* it_;

  //! Point cloud message to publish.
  sensor_msgs::PointCloud2::Ptr point_cloud_message_;
  cv_bridge::CvImage rgb_image_;
  cv_bridge::CvImage depth_image_;

  pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_;


  void timerCallback(const ros::TimerEvent& );
  bool publish();
protected:

  Eigen::Matrix3d intrins_;
  int width_;
  int height_;
  std::string pc_file_path_;
  std::string point_cloud_topic_;
  std::string depth_topic_;
  std::string rgb_topic_;

  double rate_;
  std::string frame_;

  //! Duration between publishing steps.
  ros::Duration update_duration_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

};

#endif // POINT_CLOUD_PUBLISHER_HPP
