#include "ros_perception_utils/pcl_visibility.hpp"
Visibility::Visibility()
{
  cam_params_.setIdentity();
}
Visibility::~Visibility()
{}
bool Visibility::readIntrinsicsAndCam2WorldPoses(const std::string& filepath)
{
  cv::FileStorage fs(filepath+"/depth_cam2world_poses.yaml",cv::FileStorage::READ);
  cv::Mat cvMat;
  fs["CENTROID"]>>cvMat;
  cv::cv2eigen(cvMat,marker2object_);
  // iterate through a sequence using FileNodeIterator
  for(int i=0;;i++)
  {
      std::ostringstream ss;
      ss<<i;

      std::string transform_flag = "TRANSFORM " + ss.str();
      std::string flag = "Flag " +ss.str();
      //Read transform
      if(fs[transform_flag].isNone())
      {
          std::cout<<"MeshVisibility::readCam2WorldPoses: "<< i <<" poses"<<std::endl;
          break;
      }
      cv::Mat cam2world;
      fs[transform_flag]>>cam2world;
      Eigen::Matrix4d cam2object;
      Eigen::Matrix4d eigenMat;
      cv::cv2eigen(cam2world,eigenMat);
      cam2object = marker2object_ *eigenMat; //cam to object centroid ( also world)
      cam2world_poses_.push_back(cam2object);
  }

  fs.release();
  std::ifstream file ;
  std::string cam_info_file = filepath +"/texture_camera.txt";
  double val[9];
  file.open(cam_info_file.c_str());
  for(int i=0;i<9;i++)
  {
      file>>val[i];
  }
  cam_params_(0,0)=val[0];
  cam_params_(0,2)=val[2];
  cam_params_(1,1)=val[4];
  cam_params_(1,2)=val[5];


  // std::copy(coeffs,coeffs+5,color_cam_params.coeffs);
  file>>image_width_;
  file>>image_height_;
  std::cout<<"Intrinsics: "<<cam_params_<<std::endl;


}
