#include "ros_perception_utils/visibility.hpp"
MeshVisibility::MeshVisibility():focal_x_(0),focal_y_(0),
    c_x_(0),c_y_(0),image_width_(0),image_height_(0)
{
    marker2object_.setIdentity();
}
MeshVisibility::~MeshVisibility()
{}

bool MeshVisibility::readTriMesh(const std::string mesh_fname)
{
    std::string mesh_suffix = mesh_fname.substr(mesh_fname.length() - 3, 3);
    ROS_INFO("Reading mesh file %s", mesh_fname.c_str());
    if (mesh_suffix == "ply" || mesh_suffix == "PLY")
    {
        if (!VisionUtils::readPLYFile(mesh_fname,tri_mesh_))
        {
            return false;
        }
        std::cout << "#Vertex: " << tri_mesh_.vertex_num_ << ", #Faces: " << tri_mesh_.face_num_ << std::endl;

    }
    else if (mesh_suffix == "obj" || mesh_suffix == "OBJ")
    {
        if (!VisionUtils::readOBJFile(mesh_fname,tri_mesh_))
        {
            return false;
        }
        std::cout << "#Vertex: " << tri_mesh_.vertex_num_ << ", #Faces: " << tri_mesh_.face_num_ << std::endl;
    }
}
bool MeshVisibility::readIntrinsicsAndCam2WorldPoses(const std::string filepath)
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
        Eigen::Matrix4d pose;
        Eigen::Matrix4d eigenMat;
        cv::cv2eigen(cam2world,eigenMat);
        pose = marker2object_ *eigenMat; //cam to object centroid
        cv::eigen2cv(pose,cam2world);
        glm::mat4 trans;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                trans[j][i] = cam2world.at<double>(i,j);

            }
        cam2world_poses_.push_back(trans);
    }

    fs.release();
    std::ifstream file ;
    std::cout<<"read camera parameters"<<std::endl;
    std::string cam_info_file = filepath +"/texture_camera.txt";
    float val[9];
    file.open(cam_info_file.c_str());
    for(int i=0;i<9;i++)
    {
        file>>val[i];
    }
    focal_x_=val[0];
    c_x_=val[2];
    focal_y_=val[4];
    c_y_=val[5];


    // std::copy(coeffs,coeffs+5,color_cam_params.coeffs);
    file>>image_width_;
    file>>image_height_;

    printf("%15.8e\t %15.8e\t %15.8e\t\n", focal_x_, 0.0f, c_x_);
    printf("%15.8e\t %15.8e\t %15.8e\t\n", 0.0f,focal_y_, c_y_);
    printf("%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 1.0f);
    printf("%d\t %d\t\n",image_width_,image_height_);

    //computePerspectiveMatrix

    transform_perspective_ = glm::mat4(0);
    transform_perspective_[0][0] = focal_x_ / c_x_;
    transform_perspective_[1][1] = focal_y_ / c_y_;
    transform_perspective_[2][2] = (kNear + kFar) / (kNear - kFar);
    transform_perspective_[2][3] = -1;  // glm matrix is in column-major
    transform_perspective_[3][2] = 2 * kFar * kNear / (kNear - kFar);
    //    image_buffer_arr_ = new float*[image_height_];
    //    for(int i=0;i<image_height_;i++)
    //    {
    //        image_buffer_arr_[i] = new float[3*image_width_];
    //    }

}


glm::mat4 MeshVisibility::computeTransformationForFrame(int frame_idx)
{
    // In the 3D world space, the model is fixed while the camera is moving under different poses.
    // However, in OpenGL space, the camera position is fixed while the model is transformed inversely.

    glm::mat4 trans_model = glm::inverse(cam2world_poses_[frame_idx]); //extrinsics
    //std::cout<<glm::to_string(cam2world_poses_[frame_idx])<<std::endl;
    glm::mat4 trans_scale = glm::mat4(1.0);
    // trans_scale[0][0] = trans_scale[1][1] = trans_scale[2][2] = scale_factor_; // scale if you want

    // In 3D model (world) coordinate space, +x is to the right, +z is to the inside of the screen, so +y is to the bottom.
    glm::mat4 trans_camera = glm::lookAt(glm::vec3(0, 0, 0),  // In OpenGL camera position is fixed at (0,0,0)
                                         glm::vec3(0, 0, 1),                                   // +z, position where the camera is looking at
                                         glm::vec3(0, -1, 0)                                   // +y direction
                                         );
    //std::cout << std::endl << std::endl << glm::to_string(glm::transpose(trans_camera)) << std::endl;
    // Note for the order of multiplication of different matrices.
    return transform_perspective_ * trans_camera * trans_scale * trans_model;
}

void MeshVisibility::initModelDataBuffer()
{
    image_buffer_.initNew(image_width_, image_height_);
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &EBO_);

    glBindVertexArray(VAO_);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, tri_mesh_.vertices.size() * sizeof(Vertex), &tri_mesh_.vertices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, tri_mesh_.faces.size() * sizeof(unsigned int), &tri_mesh_.faces[0], GL_STATIC_DRAW);

    // Set attributes for vertices
    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // vertex colors
    glEnableVertexAttribArray(1);
    // The first parameter is the offset, while the last one is the offset variable name and struct name which
    // must be exactly the same as that used in struct Vertex.
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, color));  // name of 'color' variable in struct Vertex

    // texture uv coordinates

    if (tri_mesh_.vtx_texture)
    {
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              (void *)offsetof(Vertex, uv));  // name of 'color' variable in struct Vertex
    }

    // normals
    if (tri_mesh_.vtx_normal)
    {
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              (void *)offsetof(Vertex, normal));  // name of 'normal' variable in struct Vertex
    }


    //cv::imwrite("tst.png",tri_mesh_.texture_atlas);

    cv::flip(tri_mesh_.texture_atlas, tri_mesh_.texture_atlas, 0);

    glGenTextures(1, &texture0_);
    glBindTexture(GL_TEXTURE_2D, texture0_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // NOTE: this will generate texture on GPU. It may crash on a GPU card with insufficient memory if
    // using a super large texture image.
    glTexImage2D(GL_TEXTURE_2D,  // Type of texture
                 0,                       // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,                  // Internal colour format to convert to
                 tri_mesh_.texture_atlas.cols,    // Image width  i.e. 640 for Kinect in standard mode
                 tri_mesh_.texture_atlas.rows,   // Image height i.e. 480 for Kinect in standard mode
                 0,                       // Border width in pixels (can either be 1 or 0)
                 GL_BGR,                  // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,        // Image data type
                 tri_mesh_.texture_atlas.ptr());   // The actual image data itself

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindVertexArray(0);
}
void MeshVisibility::draw()
{
    // draw mesh
    glViewport(0, 0, image_width_, image_height_);
    glBindVertexArray(VAO_);
    glDrawElements(GL_TRIANGLES, int(tri_mesh_.faces.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
void MeshVisibility::deallocate()
{
    // optional: de-allocate all resources once they've outlived their purpose:
    glDeleteVertexArrays(1, &VAO_);
    glDeleteBuffers(1, &VBO_);
    glDeleteBuffers(1, &EBO_);
}

void MeshVisibility::prepareImageBuffer()
{
    image_buffer_.bindForWriting();
}





void MeshVisibility::extractImageBuffer()
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    image_buffer_.bindForReading();
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    // In 'image_buffer_arr_', the channel 0 is depth value, while channel 2 is vertex index.
    // This is set in the fragment shader 'depth.frag'.
    glReadPixels(0, 0, image_width_, image_height_, GL_RGB, GL_FLOAT,image_buffer_arr_);

}


//! Save only indices of visible vertices into a binary file.
void MeshVisibility::saveVisibleVertices2Binary(const std::string filename)
{

    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, image_width_, image_height_, GL_RGB, GL_FLOAT,image_buffer_arr_);

    // Put indices of visible vertices into an array and save it in binary
    std::unordered_set<int> set_vlist;
    for (int i = 0; i < image_height_; i++)
    {
        for (int j = 0; j < image_width_; j++)
        {
            // Channel 2 is vertex index, which is set in fragment shader.
            float vidx = image_buffer_arr_[image_height_ - i - 1][3 * j + 2];
            // int idx = (vidx > 0 && vidx < 1) ? -1 : int(vidx);
            if (vidx == 0 || vidx >= 1)
                set_vlist.insert(int(vidx));
        }
    }
    std::vector<int> vec_vlist(set_vlist.begin(), set_vlist.end());
    FILE *fout = fopen(filename.c_str(), "wb");
    int num = int(vec_vlist.size());
    fwrite(&num, sizeof(int), 1, fout);
    fwrite(&vec_vlist[0], sizeof(int), num, fout);
    fclose(fout);
}

void MeshVisibility::saveVisibilityImage2Binary(const std::string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, image_width_, image_height_, GL_RGB, GL_FLOAT, image_buffer_arr_);
    std::vector<int> visible_vlist;
    visible_vlist.reserve(image_width_ * image_height_);
    for (int i = 0; i < image_height_; i++)
    {
        for (int j = 0; j < image_width_; j++)
        {
            // Channel 2 is vertex index, which is set in fragment shader.
            float vidx = image_buffer_arr_[image_height_ - i - 1][3 * j + 2];
            // If a pixel position doesn't denote any visible vertex, its value is set as -1.
            // Sometimes a pixel's vertex index is a floating value between (0,1), set as as invisible.
            int idx = ((vidx > 0 && vidx < 1) || vidx < 0) ? -1 : int(vidx);
            visible_vlist.push_back(vidx);
        }
    }
    FILE *fout = fopen(filename.c_str(), "wb");
    int num = int(visible_vlist.size());  // save #visible vertices at first
    fwrite(&num, sizeof(int), 1, fout);
    fwrite(&visible_vlist[0], sizeof(int), num, fout);
    fclose(fout);
}

void MeshVisibility::saveDepth2PNG(const std::string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, image_width_, image_height_, GL_RGB, GL_FLOAT, image_buffer_arr_);

    cv::Mat mat(image_height_, image_width_, CV_16U);
    for (int i = 0; i < image_height_; i++)
    {
        for (int j = 0; j < image_width_; j++)
        {
            // The extracted data from shader is upside down to the PNG image so we
            // need to reverse the vertical coordinate.
            unsigned short &d = mat.at<unsigned short>(image_height_ - i - 1, j);
            float depth = image_buffer_arr_[i][3 * j];
            d = (unsigned int)(depth * 5000);  // scale depth data to see it more clearly
        }
    }
    cv::imwrite(filename, mat);
}
void MeshVisibility::saveColor2PNG(const std::string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_COLOR);
    glReadPixels(0, 0, image_width_, image_height_, GL_RGB, GL_FLOAT,image_buffer_arr_);

    cv::Mat mat(image_height_, image_width_, CV_8UC3);
    for (int i = 0; i < image_height_; i++)
    {
        for (int j = 0; j < image_width_; j++)
        {
            // The extracted data from shader is upside down to the PNG image so we
            // need to reverse the vertical coordinate.
            cv::Vec3b &bgr = mat.at<cv::Vec3b>(image_height_ - i - 1, j);  // opencv uses BGR instead of RGB
            bgr[2] = (unsigned char)(image_buffer_arr_[i][3 * j] * 255);  // rgb data is float value in [0,1]
            bgr[1] = (unsigned char)(image_buffer_arr_[i][3 * j + 1] * 255);
            bgr[0] = (unsigned char)(image_buffer_arr_[i][3 * j + 2] * 255);
        }
    }
    cv::imwrite(filename, mat);
}
