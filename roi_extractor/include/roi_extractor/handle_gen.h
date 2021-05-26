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
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

// PCL Header
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

// KDL header
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

const double FINGER_LEN_X     = 0.02;
const double FINGER_LEN_Y     = 0.02;
const double FINGER_LEN_Z     = 0.05;
const double HAND_BITE        = 0.15;
const int    NUM_FOR_SAMPLING = 10;

enum door_type{ STICK , HANDLE};

class handle_sampler    
{
    public:
    handle_sampler(ros::NodeHandle &_nh,double _hz);
    ~handle_sampler(){};

    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){return roi_cloud_;};
    inline void setGraspViusual(bool _sig){grasp_visualization_ = _sig;};

    void InitRobotKinematics(KDL::JntArray _nominal, KDL::Chain _robot_chain);

    geometry_msgs::Pose getSolution(unsigned int _objNum);


    struct graspCandidate{
        unsigned int door_type_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_;
        std::vector<geometry_msgs::Pose> result_candidate;
    };

    private:

    ///////////// function ///////////////////////////////
    void reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_filter_cloud(Eigen::Vector4f,Eigen::Vector4f);
    void obj_visualization();
    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

    //////////// Variable /////////////////////////////////
    ros::Rate rate_;
    ros::Publisher obj_cloud_pub_;
    ros::Subscriber yolo_detection_sub_;
    ros::Subscriber kinect_cloud_sub_;

    Eigen::Matrix4f T_BC_;

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;

    KDL::Chain FK_chain_;
    KDL::JntArray robot_joint_val_;
    KDL::Frame T_BC_Frame;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualization_cloud_;
    
    std::vector<graspCandidate> result_;

    bool grasp_visualization_;
    int  detected_obj_num_;
};