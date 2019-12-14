#ifndef VISIBILITY_HPP
#define VISIBILITY_HPP
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext.hpp>

//#include "shader.h"
#include "ros_perception_utils/gbuffer.h"
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

#include "ros_perception_utils/vision_utils.hpp"

class Visibility
{
public:
  Visibility();
  bool readIntrinsicsAndCam2WorldPoses(const std::string& filepath);
  ~Visibility();
private:
  TriMesh tri_mesh_;
  std::vector<Eigen::Matrix4d> cam2world_poses_;
  Eigen::Matrix4d marker2object_;
  Eigen::Matrix3d cam_params_;
  int image_width_;
  int image_height_;
};



#endif // VISIBILITY_HPP
