#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "RRTFunction.h"
#include "YamlConfig.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/node_handle.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

// for output joint trajectory
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"

// for input data
// #include "arm_motion_planner/plan_arm_motion.h"
#include "task_assembly/door_open_planner.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

// for generating trajectory
#include "/home/dyros/trajectory_smoothing/include/Trajectory.h"
#include "/home/dyros/trajectory_smoothing/include/Path.h"



#include <ctime>

  // random with microsec

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;


class ArmPlanner
{

public: 
	ArmPlanner(YAMLConfig yaml_config);
	~ArmPlanner();

public:
	trajectory_msgs::JointTrajectory compute();
	void initModel();
	// void initializeData(arm_motion_planner::plan_arm_motion::Request &req);

// variable Setting

	// Manipulator joint states
	struct jointState {
		VectorXd qInit_;
		VectorXd qGoal_;
	};
	// Mobile base joint states
	struct mobileState {
		VectorXd qInit_;
		Matrix3d rotInit_;
	};
	struct jointLimit {
		VectorXd lower_;
		VectorXd upper_;
		VectorXd lower_rad_;
		VectorXd upper_rad_;
	};


	//// RBDL ////
	Model rbdl_model_;
	jointState joint_state_;

	RRT rrt_;

	Robotmodel rrt_model_;
	MatrixXd joint_target_, joint_target2_;

	// Trajectory library // 
	bool interpolate_path_;
	Trajectory *trajectory_generator_;
	double duration_;
	VectorXd maxAcceleration;
	VectorXd maxVelocity;
	list<VectorXd> wayPoints;


	double playTime_ ;

	//////////////////// Service Server Parameters ///////////////////////
	YAMLConfig config_;
	unsigned int end_effector_id_;
	unsigned int arm_base_frame_id_;
	Vector3d end_effector_com_;
	Vector3d arm_base_frame_pos_;
	int nb_of_joints_;
	std::vector<int> body_id_collision_;
	std::vector<Vector3d> body_com_position_;
	jointLimit joint_limit_;

	Transform3d target_pose_, init_pose_;
	mobileState mobile_pose_;
	bool constrain_pose_;
	trajectory_msgs::JointTrajectory output_trajectory_;
	trajectory_msgs::JointTrajectoryPoint trajectory_point_;

	Vector3d base_frame_pos_;

	/////////////////////////////////////////////////////////////////////////

	bool setupRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);
	bool setupCRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);

	bool solveRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd& joint_target);
	bool solveCRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd& joint_target);
	bool solveIK(Transform3d pose, Robotmodel &model);
	std::string urdf_param_;
	
	double fRand(double min, double max)
	{
	double f = (double)rand() / RAND_MAX;
	return min + f * (max - min);
	};

	static const Matrix3d Vec_2_Rot(const Vector3d& vec) {
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else {
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle))* V* V;
		}
		return Rot;
	};
};
#endif