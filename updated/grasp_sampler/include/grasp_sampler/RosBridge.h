
#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/node_handle.h>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include <fiducial_msgs/FiducialTransformArray.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "Utils.h"
#include <iostream>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


class RosBridge
{
public:
    RosBridge(ros::NodeHandle nh_, double hz_);
    ~RosBridge();

    void qrtransformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & g);

    
    void ReadQRCode();
    void BaseTransform();
    void WriteAngle(double door_angle);
    void WriteDoorPosition(Eigen::Vector3d pos);

    fiducial_msgs::FiducialTransformArray qr_trans;


private:

    ros::Publisher door_angle_pub_ ; 
    ros::Publisher door_pos_pub_ ; 
    ros::Subscriber qr_transform_sub_;


    Eigen::Quaterniond q1;
    Eigen::Quaterniond q2;
    Eigen::Matrix3d R1, R2, R3;
    Eigen::Vector3d Euler; 
     

    ros::Rate rate_;

    std_msgs::Float64 doorangle;
    geometry_msgs::Point doorpos;
    float doorangle_;
};

#endif
