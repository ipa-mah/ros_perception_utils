/*!
 *****************************************************************
 * @file aruco_marker.hpp
 *****************************************************************
 *
 * @note Copyright (c) 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * @note Project name: ros_perception_utils
 * @author Author: Manh Ha Hoang
 *
 * @date Date of creation: 08.2019
*/
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>


class Marker
{
public:
    struct charuco_params {
        int square_x;
        int square_y;
        float square_length;
        float marker_length;
        int dictionary_id;
        int border_bits;
        int margins;
    };
    struct aruco_params {
        int markers_x;
        int markers_y;
        int marker_length;
        int marker_separation;
        int dictionary_id;
        int margins;
        int border_bits;
    };
    struct aruco_pose {
        cv::Mat rot;
        cv::Mat trans;
    };
    typedef cv::Ptr<Marker> Ptr;
    // Create Marker class with Charuco
    Marker(int square_x,int square_y,float square_length,float marker_length,int dictionary_id,int border_bits,int margins );
    // Create Maker class with Aruco
    Marker(int markers_x,int markers_y,float marker_length,int dictionary_id,int marker_separation, int border_bits,int margins );

    ~Marker();
    void setCameraMatrix(const cv::Mat& camera_matrix) {camera_matrix_ = camera_matrix;}
    void setCameraParameters(const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs);
    /*Load configuration of charuco and aruco board*/
    unsigned long LoadParameters(std::string directory_and_filename);
    /* Create an aruco board with desired configuration*/
    unsigned long createChaRucoMarker();
    /* Create an aruco board with desired configuration*/
    unsigned long createArucoMarker(cv::Mat& board_images);
    /*Estimate transformation matrix from charuco board coordinate to camera coordinate,
     * if found return true, else return false
     */
    bool estimatePoseCharuco(const cv::Mat& frame,Eigen::Matrix4d& camera_to_marker);
    bool estimatePoseAruco(const cv::Mat& frame, Eigen::Matrix4d& camera_to_marker);
    ////
    /// \brief addCube add 3D bounding box in image
    /// \param img : input image
    ///
    void addCube(cv::Mat& img, const aruco_pose& pose);
private:

    charuco_params charuco_params_;
    aruco_params aruco_params_;
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
    cv::Ptr<cv::aruco::Board> board_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

};
