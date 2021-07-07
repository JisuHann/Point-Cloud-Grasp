#include "../include/grasp_sampler/arm_interface.h"

arm_controller_interface::arm_controller_interface(ros::NodeHandle & _nh, YAMLConfig &_config,double _hz) : 
nh_(_nh),
hz_(_hz) 
{
    arm_kinematics_solver_ = new kinematics_sovler(_config);
    _nh.param("/urdf_param", arm_kinematics_solver_->urdf_param_, std::string("/robot_description"));

    door_handle_sampler_ = new handle_sampler(nh_,hz_);

    service_ = nh_.advertiseService("/plane_door_grasp_motion",&arm_controller_interface::calcGraspPose,this);
    nb_of_joint_ = arm_kinematics_solver_->getNbOfJoint();
}


bool arm_controller_interface::calcGraspPose(task_assembly::door_open_planner::Request  &req,task_assembly::door_open_planner::Response &res)
{
    std::cout<<"your in srv processing"<<std::endl;
    // wait until convergence rate less than 1.5
    //while(!door_handle_sampler_->IsGraspStable());

    geometry_msgs::Pose graspHandleResult;
    graspHandleResult = door_handle_sampler_->getGraspResult();
    
    //print grasp position

    std::cout<<"grasp configuration  x : "<<graspHandleResult.position.x
    <<"  y: "<<graspHandleResult.position.y<<"   z: "<<graspHandleResult.position.z<<std::endl;

    arm_kinematics_solver_->initializeData(req);

    res.grasp_pose = graspHandleResult;
    if(!arm_kinematics_solver_->solveIK(graspHandleResult))
    {
        std::cout << "IK not Solved !!!"<<std::endl;
        return false;
    }

    //res.grasp_pose = graspHandleResult;
    std::cout<<"hello "<<std::endl;
    return true;
}