
// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <ros/package.h>
// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>


// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "ros_perception_utils/gl_visibility.hpp"
#include "ros_perception_utils/shader.h"
#include "ros_perception_utils/vision_utils.hpp"
GLFWwindow* window;
const int kWidth = 1920;
const int kHeight = 1080;
// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
  // make sure the viewport matches the new window dimensions; note that width and
  // height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

bool initGLFW()
{
  if (!glfwInit())
  {
    std::cout << "Failed to initialize GLFW" << std::endl;
    return false;
  }
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // We don't want the old OpenGL
  //if (program_mode_ != RENDER_MODEL)
  //  glfwWindowHint(GLFW_VISIBLE, false);  // hide window after creation
  // Open a window and create its OpenGL context
  window = glfwCreateWindow(kWidth, kHeight, "RenderingWindow", NULL, NULL);
  if (window == NULL)
  {
    std::cout << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version "
                 "of the tutorials."
              << std::endl; // this is from some online tutorial
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // Initialize GLEW
  glewExperimental = true;  // Needed for core profile
  if (glewInit() != GLEW_OK)
  {
    fprintf(stderr, "Failed to initialize GLEW\n");
    getchar();
    glfwTerminate();
    return false;
  }

  // Ensure we can capture the escape key being pressed below
  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

  // Dark black background
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  return true;
}
void runRenderMode( MeshVisibility* mesh)
{
  Shader myshader;
  std::string render_vert = ros::package::getPath("ros_perception_utils")+"/shaders/rendermode.vert";
  std::string render_frag = ros::package::getPath("ros_perception_utils")+"/shaders/rendermode.frag";
  myshader.LoadShaders(render_vert.c_str(), render_frag.c_str());
  myshader.setInt("texture_sampler", 0);
  int frame_idx = 0;
  do
  {
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    myshader.useProgram();
    myshader.setFloat("near", kNear);
    myshader.setFloat("far", kFar);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    {
      frame_idx++;
      if (frame_idx >= mesh->cam2world_poses_.size())
        frame_idx = 0;
      // cout << "Frame " << frame_idx + kStartFrameIdx << endl;
    }
    else if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    {
      frame_idx--;
      if (frame_idx < 0)
        frame_idx = mesh->cam2world_poses_.size() - 1;
      //std::cout << "Frame " << frame_idx  << std::endl;
    }
    myshader.setMat4("transform", mesh->computeTransformationForFrame(frame_idx));
    // std::cout<<glm::to_string( mesh.computeTransformationForFrame(frame_idx))<<std::endl;
    // Press 'C' to show color and 'D' to show depth
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
    {
      myshader.setInt("flag_show_color", true);
      myshader.setBool("flag_show_texture", false);
      // std::cout << "color " << frame_idx  << std::endl;

    }

    else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
      myshader.setBool("flag_show_color", false);
      myshader.setBool("flag_show_texture", false);
      //  std::cout << "depth " << frame_idx  << std::endl;

    }

    else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && mesh->tri_mesh_.vtx_texture)
    {
      myshader.setBool("flag_show_color", false);
      myshader.setBool("flag_show_texture", true);
      //  std::cout << "texture " << frame_idx  << std::endl;    std::cout<<"sda"<<std::endl;


    }

    if (mesh->tri_mesh_.vtx_texture)
    {
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, mesh->texture0_);
    }

    mesh->draw();
    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    glfwSwapBuffers(window);
    glfwPollEvents();
  }  // Check if the ESC key was pressed or the window was closed
  while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
  myshader.deleteProgram();
}

void runSaveMode(std::string output_path, MeshVisibility* mesh)
{
  Shader myshader;
  std::string savemode_vert = ros::package::getPath("ros_perception_utils")+"/shaders/savemode.vert";
  std::string savemode_frag = ros::package::getPath("ros_perception_utils")+"/shaders/savemode.frag";
  myshader.LoadShaders(savemode_vert.c_str(), savemode_frag.c_str());
  myshader.setInt("texture_sampler", 0);

  for(int frame_idx=0;frame_idx<mesh->cam2world_poses_.size();frame_idx++)
  {
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    myshader.useProgram();
    myshader.setFloat("near", kNear);
    myshader.setFloat("far", kFar);
    myshader.setBool("flag_show_color", true);
    myshader.setBool("flag_show_texture", false);
    //prepare image buffer
    mesh->prepareImageBuffer();
    // NOTE: depth test and clear function MUST be put after binding frame buffer and also need to
    // run for each frame, otherwise the extracted depth and color data will not have depth test.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    myshader.setMat4("transform", mesh->computeTransformationForFrame(frame_idx));
    if (mesh->tri_mesh_.vtx_texture)
    {
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, mesh->texture0_);
      myshader.setInt("texture_sampler", 0);
    }

    mesh->draw();
    mesh->extractImageBuffer();

    std::string output_fname = output_path + mesh->getFilename(frame_idx);
    mesh->saveColor2PNG(output_fname + ".rcolor.png");
    mesh->saveDepth2PNG(output_fname + ".rdepth.png");
    std::cout<<frame_idx<<std::endl;
   // mesh->saveVisibleVertices2Binary(output_fname + ".visibility.txt");

    // This is to save the entire visibility image into binary.
    // mesh->saveVisibilityImage2Binary(output_fname + ".visimage.txt");

    // // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    // glfwSwapBuffers(window);
    // glfwPollEvents();
  }
  myshader.deleteProgram();


}




bool readVisibilityFile(const std::string& filename, std::vector<int>& visible_vertices)
{
  FILE* fin = fopen(filename.c_str(), "rb");
  if (fin == NULL)
  {
    ROS_INFO("ERROR: cannot read visibility file %s", filename.c_str());
    return false;
  }
  int num = -1;
  if (fread(&num, sizeof(int), 1, fin) != 1)
    return false;
  if (num <= 0)
  {
    ROS_INFO("WARNING: number of visible vertices in file %s is <= 0", filename.c_str());
    return true;
  }
  visible_vertices.clear();
  visible_vertices.resize(num);
  if (fread(&visible_vertices[0], sizeof(int), num, fin) != size_t(num))
  {
    ROS_INFO("ERROR in reading visibility indices in file %s", filename.c_str());
    return false;
  }
  fclose(fin);
  return true;
}

int main( int argc, char** argv )
{



  std::string data_path = "/home/ipa-mah/catkin_ws/data/bagfile/";


    MeshVisibility* visibility = new MeshVisibility();
    visibility->readTriMesh(data_path+"/texture_model.obj");
    visibility->readIntrinsicsAndCam2WorldPoses(data_path);
    initGLFW();
    visibility->initModelDataBuffer();
    //runRenderMode(visibility);
    runSaveMode(data_path+"/visibility/",visibility);
    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    visibility->deallocate();

  /*
  cv::Mat depth = cv::imread(data_path+"/visibility/frame-000000.rdepth.png",2);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.is_dense = true;
  for(int i=0;i<depth.cols;i++)
    for(int j=0;j<depth.rows;j++)
    {
      float d = (float)(depth.at<unsigned short>(j,i))/5000.0f;
      pcl::PointXYZ pt;
      pt.z = d ;
      pt.x =  d * (i - 948.960205078125) / 1383.5277099609375;
      pt.y =  d * (i - 535.8566284	) / 1383.5277099609375	;
      cloud.points.push_back(pt);
    }
  cloud.width = depth.cols*depth.rows;
  cloud.height = 1;
  pcl::io::savePLYFile("cloud.ply",cloud);
  */
  return 0;
}
