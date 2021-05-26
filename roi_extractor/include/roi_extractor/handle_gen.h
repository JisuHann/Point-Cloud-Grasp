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
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

// msgs Filter

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

// PCL Header
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

// KDL header
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

class handle_sampler
{
    public:
    handle_sampler(ros::NodeHandle &_nh,double _hz);
    ~handle_sampler(){};

    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){return roi_cloud_;};

    private:

    ///////////// function ///////////////////////////////
    void reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose);

    //////////// Variable /////////////////////////////////
    ros::Rate rate_;
    ros::Subscriber yolo_detection_sub_;

    Eigen::Matrix4f T_BC_;

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;

    KDL::Chain FK_chain_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
};