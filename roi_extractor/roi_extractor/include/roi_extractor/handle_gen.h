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
#include <visualization_msgs/Marker.h>

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

// PCL Header
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h> 
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

// KDL header
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

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
    handle_sampler(ros::NodeHandle &_nh,double _hz);
    ~handle_sampler(){};

    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){return roi_cloud_;};
    inline void setGraspViusual(bool _sig){original_color_visualization_ = _sig;};

    void InitRobotKinematics(KDL::JntArray _nominal, KDL::Chain _robot_chain);

    std::vector<geometry_msgs::Pose> getSolution(unsigned int _objNum);

    void obj_visualization();


    struct graspCandidate{
        unsigned int door_type_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud_;
        std::vector<geometry_msgs::Pose> result_candidate;
    };

    private:

    ///////////// function ///////////////////////////////

    double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

    void reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_filter_cloud(Eigen::Vector4f,Eigen::Vector4f);
    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

    //////////// Variable /////////////////////////////////
    ros::Rate rate_;
    ros::Publisher roi_cloud_pub_;
    ros::Publisher handle_cloud_;
    ros::Publisher grasp_pub_;


    ros::Subscriber yolo_detection_sub_;
    ros::Subscriber kinect_cloud_sub_;

    sensor_msgs::PointCloud2 cloud_msgs_;
    visualization_msgs::Marker grasp_;
    visualization_msgs::MarkerArray grasps_;

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
    int  detected_obj_num_;
    int  target_object_num_;

    // for Load Cad Model
    int saveNum;

};