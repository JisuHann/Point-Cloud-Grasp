#include "../include/grasp_sampler/arm_interface.h"

arm_controller_interface::arm_controller_interface(ros::NodeHandle & _nh, YAMLConfig &_config,double _hz) : 
nh_(_nh),
hz_(_hz) 
{
    arm_kinematics_solver_ = new kinematics_sovler(_config);
    _nh.param("/urdf_param", arm_kinematics_solver_->urdf_param_, std::string("/robot_description"));
    _nh.param("sim_enable", this->simEnable);

    door_handle_sampler_ = new handle_sampler(nh_,hz_,this->simEnable);

    if(simEnable)
    {
        std::cout<<"Simulation On"<<std::endl;
        vrep_bridge_ = new sim_controller_interface(nh_,hz_);
    }
    else
    {
        std::cout<<"Simulation Off"<<std::endl;
        vrep_bridge_ = NULL;
    }

    service_ = nh_.advertiseService("/plane_door_grasp_motion",&arm_controller_interface::calcGraspPose,this);
    nb_of_joint_ = arm_kinematics_solver_->getNbOfJoint();
}


bool arm_controller_interface::calcGraspPose(task_assembly::door_open_planner::Request  &req,task_assembly::door_open_planner::Response &res)
{
    std::cout<<"srv"<<std::endl;

    // wait until convergence rate less than 1.5
    // threads.push_back(std::thread(&arm_controller_interface::excute_srvThread, this));

    // for(auto & thread : threads)
    // {   
    //     thread.join();
    // }

    // threads.clear();

    // while(!door_handle_sampler_->IsGraspExist())
    // {
    //     //std::cout<<"wait for stable grasp"<<std::endl;
    //     ros::spinOnce();
    // }

    geometry_msgs::Pose graspHandleResult;
    graspHandleResult = door_handle_sampler_->getGraspResult();
    
    // rgb_optical_frame to Object (T_rbO)
    Eigen::Affine3f T_OO;
    T_OO.translation() << graspHandleResult.position.x,graspHandleResult.position.y,graspHandleResult.position.z;
    T_OO.linear() = Eigen::Quaternionf(graspHandleResult.orientation.x,graspHandleResult.orientation.y,graspHandleResult.orientation.z,graspHandleResult.orientation.w).toRotationMatrix();

    //rgb_optical frame to camera_base (T_rb to cb)
    Eigen::Affine3f T_LK;
    T_LK.translation() << -0.00014467 , 0.014879, 0.000155564;
    T_LK.linear() = Eigen::Quaternionf(-0.49997,0.49875,-0.49919,0.50208).toRotationMatrix();

    Eigen::Affine3f T_BC; //

    T_OO = T_LK*T_OO;  //T_CO

     // T_BO = T_BC*T_CO

    Eigen::Translation3f trans = Eigen::Translation3f(T_OO.translation());
    Eigen::Quaternionf quat = Eigen::Quaternionf(T_OO.rotation());


    // graspHandleResult.position.x = 0.662;// trans.x();
    // graspHandleResult.position.y = -1.71e-12;//trans.y();
    // graspHandleResult.position.z = 0.81;//trans.z();

    // graspHandleResult.orientation.x = quat.x();
    // graspHandleResult.orientation.y = quat.y();
    // graspHandleResult.orientation.z = quat.z();
    // graspHandleResult.orientation.w = quat.w();

    // Test Set
    graspHandleResult.position.x = 0.662;// trans.x();
    graspHandleResult.position.y = -1.71e-12;//trans.y();
    graspHandleResult.position.z = 0.81;//trans.z();

    graspHandleResult.orientation.x = -0.0185;//quat.x();
    graspHandleResult.orientation.y = -0.691153;//quat.y();
    graspHandleResult.orientation.z = 0.02;//quat.z();
    graspHandleResult.orientation.w = 0.722213;//quat.w();


    std::cout<<"your in srv processing"<<std::endl;

    std::cout<<"The Solution is ... : "<<graspHandleResult.position.x
    <<"  y: "<<graspHandleResult.position.y<<"   z: "<<graspHandleResult.position.z<<std::endl;

    arm_kinematics_solver_->initializeData(req,graspHandleResult);
    res.joint_trajectory = arm_kinematics_solver_->generateTrajectory();

    std::string output_message;    
    if (res.joint_trajectory.points.empty())
    {
        output_message = "Planning failed";
    }
    else
    {
        output_message = "Planning success";
    }
    ROS_INFO("sending back response: [%s]", output_message.c_str() ); // res.q_trajectory.joint_names[0].c_str()

    return true;

    //res.grasp_pose = graspHandleResult;
}

// void arm_controller_interface::excute_srvThread()
// {   
//     while(!door_handle_sampler_->IsGraspExist())
//     {
//         //std::cout<<"wait for stable grasp"<<std::endl;
//         ros::spinOnce();
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }


