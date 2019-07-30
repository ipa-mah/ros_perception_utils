cmake_minimum_required(VERSION 2.8.3)
project(ros_perception_utils)
#include(ExternalProject)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(catkin_RUN_PACKAGES
    message_generation
    roscpp
    roslaunch
    roslib
    std_msgs)
# set catkin packages that are needed when building the code
set(catkin_BUILD_PACKAGES ${catkin_RUN_PACKAGES} cmake_modules )


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES})
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
find_package(GLEW REQUIRED)

find_package(glfw3 3.3 REQUIRED)
FIND_PACKAGE(GLM REQUIRED)
MESSAGE("GLM DIR : ${GLM_DIR}")
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_service_files(
   FILES
   MeshVisibility.srv
)
generate_messages(
    DEPENDENCIES std_msgs
)
catkin_package(
    INCLUDE_DIRS common/include
    CATKIN_DEPENDS ${catkin_RUN_PACKAGES}
)
include_directories(
    common/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)
#add_subdirectory(elibs)
#include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/elibs/glm)
#link_directories( ${CMAKE_SOURCE_DIR}/elibs/glm)
## Class helper


file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/common/src/*.cpp")
add_library(${PROJECT_NAME} "${SOURCES}")

add_executable(${PROJECT_NAME}_node ros/mesh_visibility_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} glfw -lpthread -lm )



add_executable(opengl_opencv_node ros/opengl_opencv.cpp)
add_dependencies(opengl_opencv_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(opengl_opencv_node ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} glfw -lpthread -lm )



