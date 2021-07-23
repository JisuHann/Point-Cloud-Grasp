#pragma once

// Extern library
#include "pcl_headers.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// custom msgs
#include "task_assembly/BoundingBoxes3d.h"

// msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

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
    ~sim_controller_interface(){};

    void joint_cb(const sensor_msgs::JointStateConstPtr& msg);
    void sim_status_cb(const std_msgs::Int32ConstPtr& msg);
    void sim_time_cb(const std_msgs::Float32ConstPtr& msg);
    void vrepStart();
    void vrepStop();

    void vrepStepTrigger();
    void vrepEnableSyncMode();
    void sim_step_done_cb(const std_msgs::BoolConstPtr& msg);

    void set_desired_q(std::vector<float> dq);
    void set_exec_time(float t);
    void read_vrep();
    void compute();
    void write_vrep();
    void wait();

    void sim_create_points(const std_msgs::Float32MultiArrayConstPtr& depth_img);
    void sim_image_cb(const sensor_msgs::ImageConstPtr& img);

    private :
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

    // sim status
    ros::Rate rate_;

    bool sim_step_done_;
    float sim_time_; // from v-rep simulation time
    int tick;

    bool is_first_run;
    double start_time;
    double current_time;
    double final_time;    
    int vrep_sim_status;
    float exec_time_;

    // manipulator
    Eigen::VectorXd q_;
    Eigen::VectorXd q_dot_;
    sensor_msgs::JointState joint_cmd_;
    Eigen::VectorXd desired_q_;
    Eigen::VectorXd target_q_;
    Eigen::VectorXd init_q_;
    Eigen::VectorXd goal_q_err_;

    // sim date
    cv::Mat rgb_img_;
};

// cubic spline 

static double cubic(double time,     ///< Current time
                double time_0,   ///< Start time
                double time_f,   ///< End time
                double x_0,      ///< Start state
                double x_f,      ///< End state
                double x_dot_0,  ///< Start state dot
                double x_dot_f   ///< End state dot
                )
{
    double x_t;

    if (time < time_0)
    {
    x_t = x_0;
    }
    else if (time > time_f)
    {
    x_t = x_f;
    }
    else
    {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

        + (3 * total_x / total_time2
            - 2 * x_dot_0 / total_time
            - x_dot_f / total_time)
        * elapsed_time * elapsed_time

        + (-2 * total_x / total_time3 +
            (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time * elapsed_time;
    }

    return x_t;
}

static Eigen::Vector3d QuinticSpline(
                    double time,       ///< Current time
                    double time_0,     ///< Start time
                    double time_f,     ///< End time
                    double x_0,        ///< Start state
                    double x_dot_0,    ///< Start state dot
                    double x_ddot_0,   ///< Start state ddot
                    double x_f,        ///< End state
                    double x_dot_f,    ///< End state
                    double x_ddot_f )  ///< End state ddot
{
    double a1,a2,a3,a4,a5,a6;
    double time_s;

    Eigen::Vector3d result;

    if(time < time_0)
    {
    result << x_0, x_dot_0, x_ddot_0;
    return result;
    }
    else if (time > time_f)
    {
    result << x_f, x_dot_f, x_ddot_f;
    return result;
    }


    time_s = time_f - time_0;
    a1=x_0;
    a2=x_dot_0;
    a3=x_ddot_0/2.0;

    Eigen::Matrix3d Temp;
    Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

    Eigen::Vector3d R_temp;
    R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
        x_dot_f-x_dot_0-x_ddot_0*time_s,
        x_ddot_f-x_ddot_0;

    Eigen::Vector3d RES;

    RES = Temp.inverse()*R_temp;

    a4=RES(0);
    a5=RES(1);
    a6=RES(2);

    double time_fs = time - time_0;

    double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
    double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
    double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);


    result<<position,velocity,acceleration;

    return result;
}