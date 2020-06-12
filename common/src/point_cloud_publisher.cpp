#include <ros_perception_utils/point_cloud_publisher.hpp>

PointCloudPushliser::PointCloudPushliser(ros::NodeHandle node):
  node_handle_(node),
  it_(0),
  point_cloud_message_(new sensor_msgs::PointCloud2())
{

  ros::NodeHandle pnh("~");
  std::cout << "\n========== point_cloud_publisher Parameters ==========\n";
  if(!readParameters())
  {
    ros::requestShutdown();
  }

  // publish topics
  pc_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(point_cloud_topic_,1,true);

  // image transport
  it_ = new image_transport::ImageTransport(node_handle_);
  rgb_image_publisher_ = it_->advertise(rgb_topic_,1);
  depth_image_publisher_ = it_->advertise(depth_topic_,1);

  initialize();

}
bool PointCloudPushliser::readParameters()
{
  intrins_.setIdentity();
  bool all_read = true;
  all_read = node_handle_.getParam("pc_file_path",pc_file_path_) && all_read;
  all_read = node_handle_.getParam("point_cloud_topic",point_cloud_topic_) && all_read;
  all_read = node_handle_.getParam("depth_topic",depth_topic_) && all_read;
  all_read = node_handle_.getParam("rgb_topic",rgb_topic_) && all_read;
  all_read = node_handle_.getParam("fx",intrins_(0,0)) && all_read;
  all_read = node_handle_.getParam("fy",intrins_(1,1)) && all_read;
  all_read = node_handle_.getParam("cx",intrins_(0,2)) && all_read;
  all_read = node_handle_.getParam("cy",intrins_(1,2)) && all_read;
  all_read = node_handle_.getParam("width",width_) && all_read;
  all_read = node_handle_.getParam("height",height_) && all_read;

  all_read = node_handle_.getParam("frame",frame_) && all_read;

  double update_rate;
  all_read = node_handle_.param("rate",update_rate,1.0);
  update_duration_.fromSec(1.0 / update_rate);
  if (!all_read) {
    ROS_WARN("Could not read all parameters:\n");
    return false;
  }

  return true;
}

PointCloudPushliser::~PointCloudPushliser()
{

}
void PointCloudPushliser::initialize()
{

  if (!readPointCloudFile(pc_file_path_, frame_)) {
    ros::requestShutdown();
  }

  timer_ = node_handle_.createTimer(update_duration_, &PointCloudPushliser::timerCallback, this);
}


bool PointCloudPushliser::readPointCloudFile(const std::string& pc_file, const std::string& pc_frame_id)
{
  if (pc_file.find(".ply") != std::string::npos)
  {
    // Load .ply file.

    if (pcl::io::loadPLYFile(pc_file, point_cloud_) != 0) {
      return false;
    }

    // Define PointCloud2 message.
    pcl::toROSMsg(point_cloud_, *point_cloud_message_);
  }
  /*
  else if (pc_file.find(".vtk") != std::string::npos)
  {
    // Load .vtk file.
    pcl::PolygonMesh polygonMesh;
    pcl::io::loadPolygonFileVTK(pc_file, polygonMesh);

    // Define PointCloud2 message.
    pcl_conversions::moveFromPCL(polygonMesh.cloud, *point_cloud_message_);
  }
  */
  else if(pc_file.find(".pcd") !=std::string::npos)
  {
    // load pcd file
    if(pcl::io::loadPCDFile(pc_file,point_cloud_) !=0)
    {
      return false;
    }

    // Define PointCloud2 message.
    pcl::toROSMsg(point_cloud_, *point_cloud_message_);
  }

  else
  {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  point_cloud_message_->header.frame_id = pc_frame_id;
  point_cloud_message_->is_dense = false;
  ROS_INFO_STREAM("Loaded point cloud with " << point_cloud_message_->height * point_cloud_message_->width << " points.");
  if(point_cloud_.isOrganized())
  {

    for (std::size_t index = 0; index < point_cloud_.height * point_cloud_.width; index++)
    {
        pcl::PointXYZRGBNormal& point = point_cloud_.points[index];
        int u = static_cast<int>(index % width_);
        int v = static_cast<int>(index / width_);
        rgb_image_.image.at<cv::Vec3b>(v,u) =  cv::Vec3b(point.b, point.g, point.r);
        depth_image_.image.at<unsigned short> (v,u) = static_cast<unsigned short>(point.z * 1000);
    }
    rgb_image_.encoding = "bgr8";
    rgb_image_.header.frame_id = pc_frame_id;
    depth_image_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;;
    depth_image_.header.frame_id = pc_frame_id;
  }


  return true;
}

void PointCloudPushliser::timerCallback(const ros::TimerEvent& /*timerEvent*/) {
  if (!publish()) {
    ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
  }
}

bool PointCloudPushliser::publish() {
  point_cloud_message_->header.stamp = ros::Time::now();
  // if someone want to subscribe, then publish, otherwise holding
  if (pc_publisher_.getNumSubscribers() > 0u) {
    pc_publisher_.publish(point_cloud_message_);
    ROS_INFO_STREAM("Point cloud published to topic \"" << point_cloud_topic_ << "\".");
  }

  if(point_cloud_.isOrganized() && rgb_image_publisher_.getNumSubscribers() > 0u)
  {
    rgb_image_publisher_.publish(rgb_image_.toImageMsg());
    ROS_INFO_STREAM("RGB image published to topic \"" << rgb_topic_ << "\".");
  }

  if(point_cloud_.isOrganized() && depth_image_publisher_.getNumSubscribers() > 0u)
  {
    depth_image_publisher_.publish(depth_image_.toImageMsg());
    ROS_INFO_STREAM("Depth image published to topic \"" << depth_topic_ << "\".");
  }
  return true;
}
