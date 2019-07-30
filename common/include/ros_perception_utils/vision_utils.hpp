#ifndef VISION_UTILS_HPP
#define VISION_UTILS_HPP
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext.hpp>
#include <GL/glew.h>

#include <GLFW/glfw3.h>
struct Vertex
{
    glm::vec3 pos;// position
    glm::vec3 color; // color
    glm::vec2 uv;
    glm::vec3 normal;
    Vertex(){pos = color = glm::vec3(0);}
};


struct TriMesh
{
    std::unordered_map<std::string, int> material_names;
    std::vector<cv::Mat> texture_images;
    cv::Mat texture_atlas;
    std::vector<unsigned int> faces;
    std::vector<Vertex> vertices;
    int vertex_num_;
    int face_num_;
    std::string mesh_suffix_;
    bool vtx_normal;
    bool vtx_texture;
    TriMesh()
    {
        vtx_normal = vtx_texture = false;
        vertex_num_ = face_num_ = 0;
    }
};

namespace  VisionUtils{

bool readMTLandTextureImages(const std::string obj_folder,const std::string mtl_fname,
                             std::unordered_map<std::string, int>& material_names, std::vector<cv::Mat>& texture_images,
                             cv::Mat& texture_atlas);
bool readOBJFile(const std::string file_name, TriMesh& mesh);

///read polygon mesh (only support point + normal (no color))
bool readPLYFile(const std::string file_name, TriMesh& mesh);

bool initGLFWWindow(GLFWwindow* window);

}



#endif // VISION_UTILS_HPP
