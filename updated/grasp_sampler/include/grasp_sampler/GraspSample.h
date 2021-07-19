#pragma once

// standard Header
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>

//  ros include
#include <ros/ros.h>
#include <iostream>
#include <ros/time.h>

//  ros msgs Header
//#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <task_assembly/BoundingBoxes3d.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

//pcl headers
#include "pcl_headers.h"

// KDL header
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

const double FINGER_LEN_X     = 0.02;
const double FINGER_LEN_Y     = 0.02;
const double FINGER_LEN_Z     = 0.05;
const double HAND_BITE        = 0.15;
const int    OBJ_NUM          = 1;
const int    NUM_FOR_SAMPLING = 10;

enum door_type{ HANDLE, STICK};

class handle_sampler    
{
    public:
    handle_sampler(ros::NodeHandle &_nh,double _hz,bool simEnable);
    ~handle_sampler(){};

    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){return roi_cloud_;};
    inline void setGraspViusual(bool _sig){original_color_visualization_ = _sig;};
    inline bool IsGraspStable(){return IsGraspStable_;};
    inline bool IsGraspExist(){return IsGraspExist_;};

    inline geometry_msgs::Pose getGraspResult(){return grasp_result_;};
    void InitRobotKinematics(KDL::JntArray _nominal, KDL::Chain _robot_chain);           //if kinect hang in panda
    //double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
    std::vector<geometry_msgs::Pose> getSolution(unsigned int _objNum);

    // 감지된 모든 물체들 출력
    void obj_visualization();


    struct graspCandidate{
        unsigned int door_type_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_;
        std::vector<geometry_msgs::Pose> result_candidate;
    };

    private:

    ///////////// function ///////////////////////////////

    void reigon_cb(const task_assembly::BoundingBoxes3dConstPtr &_objpose);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_filter_cloud(Eigen::Vector4f,Eigen::Vector4f);
    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

    //////////// Variable /////////////////////////////////
    ros::Rate rate_;
    ros::Publisher roi_cloud_pub_;
    ros::Publisher handle_cloud_;
    ros::Publisher grasp_pub_;

    ros::Publisher grasp_test;

    ros::Subscriber yolo_detection_sub_;
    ros::Subscriber kinect_cloud_sub_;

    sensor_msgs::PointCloud2 cloud_msgs_;
    visualization_msgs::Marker grasp_;
    visualization_msgs::MarkerArray grasps_;

    geometry_msgs::Pose grasp_result_;

    Eigen::Matrix4f T_BC_;

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;

    KDL::Chain FK_chain_;
    KDL::JntArray robot_joint_val_;
    KDL::Frame T_BC_Frame;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualization_cloud_;
    
    std::vector<graspCandidate> result_;
    
    Eigen::Matrix3f RP;

    bool original_color_visualization_;
    bool IsGraspStable_;
    bool IsGraspExist_;
    int  detected_obj_num_;
    int  target_object_num_;

    int saveNum;
};