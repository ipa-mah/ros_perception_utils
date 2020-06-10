#include "ros_perception_utils/aruco_marker.hpp"

Marker::Marker(int square_x, int square_y, float square_length, float marker_length, int dictionary_id, int border_bits, int margins)
{
	dist_coeffs_ = cv::Mat();
	camera_matrix_ = cv::Mat();
	charuco_params_.square_x = square_x;
	charuco_params_.square_y = square_y;
	charuco_params_.border_bits = border_bits;
	charuco_params_.dictionary_id = dictionary_id;
	charuco_params_.marker_length = marker_length;
	charuco_params_.square_length = square_length;
	charuco_params_.margins = margins;
}
Marker::~Marker()
{
}
void Marker::setCameraParameters(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
{
	camera_matrix_ = camera_matrix;
	dist_coeffs_ = dist_coeffs;
	std::cout << "camera_matrix:" << std::endl << camera_matrix_ << std::endl << "distortion coefficients:" << dist_coeffs_ << std::endl;
}

unsigned long Marker::createChaRucoMarker()
{
	// Create a dictionay of board
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(charuco_params_.dictionary_id));

	// Create Charuco board
	charuco_board_ = cv::aruco::CharucoBoard ::create(charuco_params_.square_x, charuco_params_.square_y, (double)charuco_params_.square_length, (double)charuco_params_.marker_length, dictionary_);

	// Convert back to aruco structure
	board_ = charuco_board_.staticCast<cv::aruco::Board>();
	return EXIT_SUCCESS;
}
unsigned long Marker::createArucoMarker(cv::Mat& board_images)
{
	cv::Ptr<cv::aruco::GridBoard> board;
	// Define the size of board(in pixels)
	cv::Size image_size;
	image_size.width = aruco_params_.markers_x * (aruco_params_.marker_length + aruco_params_.marker_separation) - aruco_params_.marker_separation + 2 * aruco_params_.margins;
	image_size.height = aruco_params_.markers_y * (aruco_params_.marker_length + aruco_params_.marker_separation) - aruco_params_.marker_separation + 2 * aruco_params_.margins;

	// Create a dictionay of board
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(aruco_params_.dictionary_id));
	// Create aruco board
	board = cv::aruco::GridBoard::create(aruco_params_.markers_x, aruco_params_.markers_y, double(aruco_params_.marker_length), double(aruco_params_.marker_separation), dictionary);
	// Draw it on image
	board->draw(image_size, board_images, aruco_params_.margins, aruco_params_.border_bits);
	return EXIT_SUCCESS;
}

bool Marker::estimatePoseCharuco(const cv::Mat& frame, Eigen::Matrix4d& marker_to_camera)
{
	bool debug = false;
	aruco_pose pose;
	cv::Mat frame_clone = frame.clone();
	if (frame_clone.empty()) return false;
	double axisLength = 0.1;  // 10 cm
	std::vector<int> markerIds, charucoIds;
	// Pixel coordinates of makers before and after refinement
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
	std::vector<cv::Point2f> charucoCorners;

	pose.rot = cv::Mat::eye(3, 3, CV_32F);
	pose.trans = cv::Mat::zeros(0, 0, CV_32F);
	// Defaut parameters to detect marker, it can be set manually
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
	// detect markers
	cv::aruco::detectMarkers(frame_clone, dictionary_, markerCorners, markerIds, detectorParams, rejectedMarkers);

	// refined strategy to detect mores markers
	cv::aruco::refineDetectedMarkers(frame_clone, board_, markerCorners, markerIds, rejectedMarkers, camera_matrix_, dist_coeffs_);

	// interpolate charuco corners
	int interpolatedCorners = 0;
	if (markerIds.size() > 0)
		interpolatedCorners = cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, frame_clone, charuco_board_, charucoCorners, charucoIds, camera_matrix_, dist_coeffs_);

	// estimate charuco board pose
	bool validPose = false;
	if (camera_matrix_.total() != 0) validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco_board_, camera_matrix_, dist_coeffs_, pose.rot, pose.trans);
	// Convert Rodrigues rotation matrix from euclid angles
	// TESTING
	cv::Mat Rod;
	cv::Rodrigues(pose.rot, Rod);
	Rod.copyTo(pose.rot);

	//
	Eigen::Vector3d T;
	Eigen::Matrix3d R;
	T.setOnes();
	R.setIdentity();

	marker_to_camera.setIdentity();  // Set to Identity to make bottom row of Matrix 0,0,0,1
	if (pose.rot.empty() || pose.trans.empty())
	{
		std::cout << "empty pose " << std::endl;
		return false;
	}

	cv::cv2eigen(pose.rot, R);
	cv::cv2eigen(pose.trans, T);

	marker_to_camera.block<3, 3>(0, 0) = R;
	marker_to_camera.block<3, 1>(0, 3) = T;

	if (markerIds.size() > 0)
	{
		cv::aruco::drawDetectedMarkers(frame_clone, markerCorners);
	}
	if (rejectedMarkers.size() > 0)
		// cv::aruco::drawDetectedMarkers(frame, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));
		if (interpolatedCorners > 0)
		{
			cv::Scalar color;
			color = cv::Scalar(255, 0, 0);
			//   cv::aruco::drawDetectedCornersCharuco(frame, charucoCorners, charucoIds, color);
		}

	if (validPose && debug)
	{
		cv::aruco::drawAxis(frame_clone, camera_matrix_, dist_coeffs_, pose.rot, pose.trans, axisLength);
		addCube(frame_clone, pose);
		cv::namedWindow("image", CV_WINDOW_NORMAL);
		cv::imshow("image", frame_clone);
		cv::waitKey(0);
	}
	pose.rot.release();
	pose.trans.release();
	return validPose;
}
void Marker::addCube(cv::Mat& img, const aruco_pose& pose)
{
	std::vector<cv::Point3f> objectPoints;				   // coordinates of tag system
	objectPoints.push_back(cv::Point3f(0.399, 0.285, 0));  // C -0
	objectPoints.push_back(cv::Point3f(0.399, 0, 0));	  // B-1
	objectPoints.push_back(cv::Point3f(0, 0.285, 0));	  // D-2
	objectPoints.push_back(cv::Point3f(0, 0, 0));		   // A-3

	objectPoints.push_back(cv::Point3f(0.399, 0.285, 0.25));  // C1 -0
	objectPoints.push_back(cv::Point3f(0.399, 0, 0.25));	  // B1 - 1
	objectPoints.push_back(cv::Point3f(0, 0.285, 0.25));	  // D1-2
	objectPoints.push_back(cv::Point3f(0, 0, 0.25));		  // A1 -3
	std::vector<cv::Point2f> cube_imagePoints;

	// Project objectPoints into image coordinates
	cv::projectPoints(objectPoints, pose.rot, pose.trans, camera_matrix_, dist_coeffs_, cube_imagePoints);
	cv::line(img, cube_imagePoints[0], cube_imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, CV_AA);  // 0-1
	cv::line(img, cube_imagePoints[0], cube_imagePoints[2], cv::Scalar(0, 0, 255, 255), 1, CV_AA);  // 0-2
	cv::line(img, cube_imagePoints[3], cube_imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	cv::line(img, cube_imagePoints[3], cube_imagePoints[2], cv::Scalar(0, 0, 255, 255), 1, CV_AA);

	cv::line(img, cube_imagePoints[4], cube_imagePoints[5], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	cv::line(img, cube_imagePoints[4], cube_imagePoints[6], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	cv::line(img, cube_imagePoints[7], cube_imagePoints[5], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	cv::line(img, cube_imagePoints[7], cube_imagePoints[6], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	for (int i = 0; i < 4; i++)
	{
		cv::line(img, cube_imagePoints[i], cube_imagePoints[i + 4], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
	}
}
