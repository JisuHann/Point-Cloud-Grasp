#include <math.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Custom Header
#include "Kinematics.h"
#include "GraspSample.h"
#include "Utils.h"
#include "task_assembly/door_open_planner.h"

#include "VrepBridge.h"

class arm_controller_interface
{
public:
    arm_controller_interface(ros::NodeHandle & _nh,YAMLConfig &_config,double _hz);
    ~arm_controller_interface(){};

    //inline void setKinematicsModel(YAMLConfig &_config){arm_kinematics_solver_ = new kinematics_sovler(_config);};
    inline void setT_BC_Kinematics(KDL::Frame _T_BC_frame){arm_kinematics_solver_->setT_BC_frame(_T_BC_frame); };  

    // door planner server
private:
    bool calcGraspPose(task_assembly::door_open_planner::Request &req ,task_assembly::door_open_planner::Response &res);
    // void excute_srvThread();
    //ros 
    ros::ServiceServer service_;

    //custom solver
    kinematics_sovler* arm_kinematics_solver_;
    handle_sampler* door_handle_sampler_;       /// <=== Realtime?? Ehthernet Comm
    sim_controller_interface* vrep_bridge_;

    unsigned int nb_of_joint_;
    
    ros::NodeHandle nh_;
    double hz_;

    // PID gain
    Eigen::MatrixXd Kp_;  // p- gain 
    Eigen::MatrixXd Kd_;  // d-gain
    Eigen::MatrixXd Ki_;  // i-gain


    // State Feedback ///

    //////////// thread setting /////////////////////////////////////////////////
    // std::vector<std::thread> threads;
    // std::mutex mutex_;

    // Simulation activation
    bool simEnable; 
};