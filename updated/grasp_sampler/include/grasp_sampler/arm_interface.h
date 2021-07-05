#include <math.h>

// Custom Header
#include "Kinematics.h"
#include "GraspSample.h"
#include "Utils.h"
#include "task_assembly/door_open_planner.h"

class arm_controller_interface
{
public:
    arm_controller_interface(ros::NodeHandle & _nh,YAMLConfig &_config,double _hz);
    ~arm_controller_interface(){};

    //inline void setKinematicsModel(YAMLConfig &_config){arm_kinematics_solver_ = new kinematics_sovler(_config);};
    inline void setT_BC_Kinematics(KDL::Frame _T_BC_frame){arm_kinematics_solver_->setT_BC_frame(_T_BC_frame); };  

    // door planner server
    bool calcGraspPose(task_assembly::door_open_planner::Request &req);
private:
    kinematics_sovler* arm_kinematics_solver_;
    handle_sampler* door_handle_sampler_;       /// <=== Realtime?? Ehthernet Comm

    unsigned int nb_of_joint_;
    
    ros::NodeHandle nh_;
    double hz_;

    // PID gain
    Eigen::MatrixXd Kp_;  // p- gain 
    Eigen::MatrixXd Kd_;  // d-gain
    Eigen::MatrixXd Ki_;  // i-gain


    // State Feedback ///
};