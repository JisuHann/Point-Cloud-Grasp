#include "../include/grasp_sampler/arm_interface.h"

arm_controller_interface::arm_controller_interface(ros::NodeHandle & _nh, YAMLConfig &_config,double _hz) : 
nh_(_nh),
hz_(_hz) 
{
    std::cout<<"error check"<<std::endl;
    arm_kinematics_solver_ = new kinematics_sovler(_config);
    door_handle_sampler_ = new handle_sampler(nh_,hz_);

    _nh.param("/urdf_param", arm_kinematics_solver_->urdf_param_, std::string("/robot_description"));
    

    nb_of_joint_ = arm_kinematics_solver_->getNbOfJoint();
}


bool arm_controller_interface::calcGraspPose(task_assembly::door_open_planner::Request &req)
{
    // wait until convergence rate over i.5
    while(!door_handle_sampler_->IsGraspStable());

    geometry_msgs::Pose graspHandleResult = door_handle_sampler_->getGraspResult();
    
    //print grasp position
    std::cout<<"grasp configuration  x : "<<graspHandleResult.position.x
    <<"  y: "<<graspHandleResult.position.y<<"   z: "<<graspHandleResult.position.z<<std::endl;

    if(!arm_kinematics_solver_->solveIK(graspHandleResult))
    {
        std::cout << "IK not Solved !!!"<<std::endl;
        return false;
    }

    return true;
}