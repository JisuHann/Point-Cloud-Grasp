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

//srv includes
#include "task_assembly/door_open_planner.h"

// IK solver (track-ik)
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// pcl headers includes
#include "pcl_headers.h"

// standard header
#include <iomanip>
#include <iostream>
#include <map>
#include <random>

// franka panda Header

using namespace RigidBodyDynamics;

class kinematics_sovler
{
    public:
    kinematics_sovler(YAMLConfig &config);
    ~kinematics_sovler(){};
   
    // Eigen::Matrix4f getCurT_BC();

    void initializeData(task_assembly::door_open_planner::Request &req);
    bool solveIK(geometry_msgs::Pose _goal_info);
    Eigen::Matrix4f solveFK(KDL::JntArray _joint_val);
    // void solveInverseDynamics();
    // void solveFowardDynamics();

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
    //void computeReability();

    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);

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

    //params
    double eps = 5e-3;
	double num_samples = 100;
    int nb_of_sampling_points_ = 150000;
    int nb_of_clusters_ =0;
};



// preemt-rt version #1 SMP PREEMPT RT Mon Oct 14 16:57:48 KST 2019
