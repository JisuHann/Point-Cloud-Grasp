cmake_minimum_required(VERSION 3.0.2)
project(grasp_sampler)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  geometry_msgs
  task_assembly
  fiducial_msgs
  kdl_parser # IK
  trac_ik_lib  #track-IK
  urdf
)


# RBDL setting
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PCL 1.8 REQUIRED)
find_package(RBDL REQUIRED)
#find_package(Boost REQUIRED COMPONENTS date_time)   # track-IK
find_package(orocos_kdl REQUIRED)                   # track-IK

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_sampler
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  trajectory_msgs 
  task_assembly
  kdl_parser urdf
  DEPENDS system_lib Boost orocos_kdl
  DEPENDS Eigen3
)

set(SRCS
    src/Kinematics.cpp
    src/RosBridge.cpp
    src/Utils.cpp
    src/GraspSample.cpp
    src/arm_interface.cpp

    # Motion planners
    src/ArmPlanner.cpp
    src/RRTFunction.cpp
)

set(HEADS
    include/${PROJECT_NAME}/pcl_headers.h
    include/${PROJECT_NAME}/Fwd.h
    include/${PROJECT_NAME}/Kinematics.h
    include/${PROJECT_NAME}/Utils.h
    include/${PROJECT_NAME}/RosBridge.h
    include/${PROJECT_NAME}/YamlConfig.h
    include/${PROJECT_NAME}/GraspSample.h
    include/${PROJECT_NAME}/arm_interface.h

    # Motio planners
    include/${PROJECT_NAME}/ArmPlanner.h
    include/${PROJECT_NAME}/NodeTree.h
    include/${PROJECT_NAME}/RRTFunction.h
    include/${PROJECT_NAME}/RRTNode.h
)

add_library(doorPlanner 
  ${SRCS} 
  ${HEADS} 
)

link_directories(${Boost_LIBRARY_DIR} ${orocos_kdl_LIBRARIES} ${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

target_link_libraries(doorPlanner ${catkin_LIBRARIES} ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

#Server
add_executable(GraspSamplerServer src/GraspSamplerServer.cpp)
target_link_libraries(GraspSamplerServer doorPlanner ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(GraspSamplerServer task_assembly_generate_messages_cpp)

#for Test
# add_executable(testServer src/test.cpp)
# target_link_libraries(testServer doorPlanner ${catkin_LIBRARIES} yaml-cpp)
# add_dependencies(testServer task_assembly_generate_messages_cpp)

#Client
add_executable(GraspSamplerClient src/GraspSamplerClient.cpp)
target_link_libraries(GraspSamplerClient doorPlanner ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(GraspSamplerClient task_assembly_generate_messages_cpp)