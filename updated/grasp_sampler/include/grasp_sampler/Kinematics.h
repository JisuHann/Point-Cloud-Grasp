#pragma once

// ros includes
#include <ros/package.h>
#include <ros/node_handle.h>

//URDF Header
#include "YamlConfig.h"
#include <urdf/model.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

//msgs includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//srv includes
#include "task_assembly/door_open_planner.h"

// IK solver (track-ik)
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// motion planners
#include "RRTFunction.h"
#include "YamlConfig.h"

// for generating trajectory
#include "/home/dyros/trajectory_smoothing/include/Trajectory.h"
#include "/home/dyros/trajectory_smoothing/include/Path.h"

// pcl headers includes
#include "pcl_headers.h"

// standard header
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <memory>
#include <assert.h>

#include <ctime>


using namespace RigidBodyDynamics;

class kinematics_sovler
{
    public:
    kinematics_sovler(YAMLConfig &config);
    ~kinematics_sovler(){};
   
    // Eigen::Matrix4f getCurT_BC();

    // data initalize
    void initializeData(task_assembly::door_open_planner::Request &req,geometry_msgs::Pose targetPose);

    // kinematics solver
    bool solveIK(Transform3d _target_ee_pose, Robotmodel& model);
    Eigen::Matrix4f solveFK(KDL::JntArray _joint_val);

    // Dynamics solver
    // void solveInverseDynamics();
    // void solveFowardDynamics();

    // trajectory generator
    trajectory_msgs::JointTrajectory generateTrajectory();

    ////////////  inline Functions /////////////////////////
    inline Eigen::Matrix4f getT_BC_frame(){return Frame2Eigen(T_BC_Frame_);};
    inline int             getNbOfJoint(){return nb_of_joints_;};

    inline void setJointVal(KDL::JntArray _jointVal){ _jointVal = cur_joint_val_;};
    inline void setT_BC_frame(KDL::Frame _T_BC_frame){ T_BC_Frame_ = _T_BC_frame;};

    //inline KDL::JntArray getJointVal(){return cur_joint_val_;};

    // Manipulator joint states////////////////////////////////////////////
    struct jointState {
		Eigen::VectorXd qInit_;
		Eigen::VectorXd qGoal_;
	};

	struct jointLimit {
		Eigen::VectorXd lower_;
		Eigen::VectorXd upper_;
		Eigen::VectorXd lower_rad_;
		Eigen::VectorXd upper_rad_;
	};

    // URDF
    YAMLConfig config_;
    Model rbdl_model_;
    std::string urdf_param_;

    private:
    void initModel();
    void calcReachability();
    //solve Grasp problem in given scence
    bool initializeIKparam(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param);

    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

    // motion planner
    bool setupRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd& joint_target);
	bool solveRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<Eigen::VectorXd> q_goal_set, Eigen::MatrixXd& joint_target);

    // arm state
	unsigned int end_effector_id_;
	unsigned int arm_base_frame_id_;
    Eigen::Vector3d end_effector_com_;
	Eigen::Vector3d arm_base_frame_pos_;

    int nb_of_joints_;

    jointState joint_state_;
    jointLimit joint_limit_;

    // kinetmatics variable
    KDL::JntArray IK_lb, IK_ub;
	KDL::Chain IK_chain;
	KDL::Tree IK_tree;
  	urdf::Model IK_robot_model;

    KDL::JntArray cur_joint_val_;
    KDL::JntArray robot_joint_val_;

    KDL::JntArray IK_result_;
    KDL::Chain FK_chain_;

    /////////////// base to camera /////////////////
    KDL::Frame T_BC_Frame_;

    //double l2norm_;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr reachability_cloud_;

    // motion_planner //

    std::vector<int> body_id_collision_;
	std::vector<Eigen::Vector3d> body_com_position_;

    Transform3d target_pose_, init_pose_;
	RRT rrt_;

	Robotmodel rrt_model_;
	Eigen::MatrixXd joint_target_, joint_target2_;

    // Trajectory library // 
	bool interpolate_path_;
	Trajectory *trajectory_generator_;
	double duration_;
	Eigen::VectorXd maxAcceleration;
	Eigen::VectorXd maxVelocity;
	std::list<Eigen::VectorXd> wayPoints;
    double playTime_ ;

    /////////// Trajectory_msgs ////////////////
    trajectory_msgs::JointTrajectory output_trajectory_;
	trajectory_msgs::JointTrajectoryPoint trajectory_point_;

    //params
    int nb_of_sampling_points_ = 150000;
    int nb_of_clusters_ =0;
};



// preemt-rt version #1 SMP PREEMPT RT Mon Oct 14 16:57:48 KST 2019
