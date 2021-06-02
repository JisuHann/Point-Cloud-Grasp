cmake_minimum_required(VERSION 3.0.2)
project(roi_extractor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  std_msgs
  tf_conversions
  gb_visual_detection_3d_msgs
  message_filters
)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(RBDL REQUIRED)
find_package(orocos_kdl REQUIRED)  

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs   
  sensor_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES roi_extractor
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs gb_visual_detection_3d_msgs  kdl_parser urdf
  DEPENDS system_lib Boost orocos_kdl Eigen3
)

set(SRCS
  src/handle_gen.cpp
#  src/roiExtractor.cpp
  src/main.cpp
)
set(HEADS
  include/roi_extractor/handle_gen.h
)
add_library(sampling ${SRCS} ${HEADS})
target_link_libraries(sampling ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_INCLUDE_DIRS} rbdl rbdl_urdfreader yaml-cpp)

include_directories(
 include
  sampling
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

#add_executable(roi_extractor_srv src/roiExtractor.cpp ${SRCS} ${HEADS})
#target_link_libraries(roi_extractor_srv sampling)


add_executable(graspSampling src/main.cpp ${SRCS} ${HEADS})
target_link_libraries(graspSampling sampling)

