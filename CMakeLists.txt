cmake_minimum_required(VERSION 2.8.3)
project(ros_perception_utils)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(catkin_RUN_PACKAGES
    cv_bridge
    image_transport
    message_generation
    pcl_ros
    rosbag
    roscpp
    roslaunch
    roslib
    rospy
    std_msgs
    )
set(catkin_BUILD_PACKAGES ${catkin_RUN_PACKAGES} cmake_modules )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES})
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS common/include
    CATKIN_DEPENDS ${catkin_RUN_PACKAGES}
)
include_directories(
    common/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

## Class helper
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/common/src/*.cpp")

add_library(${PROJECT_NAME} "${SOURCES}")

target_link_libraries(${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${Eigen3_INCLUDE_DIRS})
#add_dependencies(${PROJECT_NAME}
#                    ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(rosbag_topics_node ros/rosbag_topics_node.cpp)
target_link_libraries(rosbag_topics_node ${PROJECT_NAME})

add_executable(point_cloud_publiser_node ros/point_cloud_publiser_node.cpp)
target_link_libraries(point_cloud_publiser_node ${PROJECT_NAME})

