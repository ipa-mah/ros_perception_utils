#ifndef MESH_VISIBILITY_H
#define MESH_VISIBILITY_H

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
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include "ros_perception_utils/vision_utils.hpp"

/* Constants */
const float kNear = 0.05f;
const float kFar = 10.0f;
const unsigned int kDigitNumInFrameName = 6; // used to create valid output filename
class MeshVisibility
{
public:
    MeshVisibility(ros::NodeHandle& node);
    ~MeshVisibility();


    /* Rendering functions */
    void initModelDataBuffer();
    void draw();
    void deallocate();

    /* Data I/O*/

    bool readIntrinsicsAndCam2WorldPoses(const std::string filepath);


    //This can be PLY file or OBJ file
    bool readTriMesh(const std::string mesh_fname);



    void saveColor2PNG(const std::string filename);
    void saveDepth2PNG(const std::string filename);
    void saveVisibleVertices2Binary(const std::string filename);
    void saveVisibilityImage2Binary(const std::string filename);


    /* Transformation */
    glm::mat4 computeTransformation(); // for debug
    glm::mat4 computeTransformationForFrame(int frame_idx);



    /* Frame buffer */
    void prepareImageBuffer();
    void extractImageBuffer();

    unsigned int texture0_;
    TriMesh tri_mesh_;
    std::vector<glm::mat4> cam2world_poses_; //extrinsics

private:
    ros::NodeHandle node_;
    float focal_x_,focal_y_,c_x_,c_y_;
    glm::mat4 transform_perspective_; // perspective transformation
    glm::vec3 camera_initial_center_;
    unsigned int VAO_, VBO_, EBO_;
    GBuffer image_buffer_;
    /* Buffers and shader-related */
    std::vector<std::vector<float>> image_buffer_arr_; // rendered image buffer
    int image_height_,image_width_;

    inline std::string getFilename(int frame_idx){
        std::string str_idx = std::to_string(frame_idx);
        return "frame-" + std::string(kDigitNumInFrameName - str_idx.length(), '0') + str_idx;
    }

};


















#endif
