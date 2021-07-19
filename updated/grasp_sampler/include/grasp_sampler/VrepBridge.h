#pragma once

// pcl Headers
#include "pcl_headers.h"

// custom msgs
#include "task_assembly/BoundingBoxes3d.h"

// msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>

// ros header
#include <ros/ros.h>
#include <ros/node_handle.h>

//image transforamtion
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>

// Paramters
#define TOTAL_DOF 9 // 9 dof robot(7 + 2 gripper)
#define SIM_DT    0.01 // 10ms
#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

const std::string JOINT_NAME[TOTAL_DOF] = {"panda_joint1","panda_joint2","panda_joint3",
"panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"};
const std::string MOBILE_JOINT_NAME[4] = {"FL_joint","FR_joint","RR_joint","RL_joint"};

class sim_controller_interface
{
    public :
    sim_controller_interface(ros::NodeHandle &_nh,double _hz);
    ~sim_controller_interface();

    // ros CallBack
    void sim_image_cb(const sensor_msgs::ImageConstPtr& img);
    void sim_create_points(const std_msgs::Float32MultiArrayConstPtr& depth_img);

    private :

    double rate_;

    // sim params
    double start_time;
    double current_time;

    // simulation msgs publisher
    ros::Publisher vrep_sim_start_pub_;
    ros::Publisher vrep_sim_stop_pub_;
    ros::Publisher vrep_sim_step_trigger_pub_;
    ros::Publisher vrep_sim_enable_syncmode_pub_;
    
    ros::Publisher vrep_joint_set_pub_;
    ros::Publisher vrep_sim_pointcloud_pub;

    // simulation msg subScribler
    ros::Subscriber vrep_joint_state_sub_;
    ros::Subscriber vrep_sim_step_done_sub_;
    ros::Subscriber vrep_sim_time_sub_;
    ros::Subscriber vrep_sim_status_sub_;

    ros::Subscriber vrep_rgb_sub;
    ros::Subscriber vrep_depth_sub;
    ros::Subscriber yolo_3d_sub;

    // manipulator

};