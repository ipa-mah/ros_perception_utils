#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
struct cha_params
{
	int square_x;
	int square_y;
	float square_length;
	float marker_length;
	int dictionary_id;
	int border_bits;
	int margins;
};
struct a_params
{
	int markers_x;
	int markers_y;
	int marker_length;
	int marker_separation;
	int dictionary_id;
	int margins;
	int border_bits;
};
struct aruco_pose
{
	cv::Mat rot;
	cv::Mat trans;
};

class Marker
{
  public:
	typedef cv::Ptr<Marker> Ptr;
	Marker(int square_x, int square_y, float square_length, float marker_length, int dictionary_id, int border_bits, int margins);
	~Marker();
	void setCameraMatrix(const cv::Mat& camera_matrix)
	{
		camera_matrix_ = camera_matrix;
	}
	void setCameraParameters(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);
	/*Load configuration of charuco and aruco board*/
	unsigned long LoadParameters(std::string directory_and_filename);
	/* Create an aruco board with desired configuration*/
	unsigned long createChaRucoMarker();
	/* Create an aruco board with desired configuration*/
	unsigned long createArucoMarker(cv::Mat& board_images);
	/*Estimate transformation matrix from charuco board coordinate to camera coordinate,
	 * if found return true, else return false
	 */
	bool estimatePoseCharuco(const cv::Mat& frame, Eigen::Matrix4d& camera_to_marker);
	////
	/// \brief addCube add 3D bounding box in image
	/// \param img : input image
	///
	void addCube(cv::Mat& img, const aruco_pose& pose);

  private:
	cha_params charuco_params_;
	a_params aruco_params_;
	cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
	cv::Ptr<cv::aruco::Board> board_;
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
};
