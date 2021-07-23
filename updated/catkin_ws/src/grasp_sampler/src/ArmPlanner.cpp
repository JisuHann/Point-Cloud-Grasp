#include "../include/grasp_sampler/ArmPlanner.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
FILE *fp1 = fopen("/home/min/catkin_ws/src/position_data.txt","w");

#define DEGREE M_PI/180.0

using std::ofstream;


ArmPlanner::ArmPlanner(YAMLConfig yaml_config):
config_(yaml_config)
{
	
}
ArmPlanner::~ArmPlanner()
{       
}
void ArmPlanner::initModel() {
	RigidBodyDynamics::Addons::URDFReadFromFile((config_.urdf_path).c_str(), &rbdl_model_, false, false);


	nb_of_joints_ = rbdl_model_.q_size;
	// Fixed Body : The value of max(unsigned int) is
	// * determined via std::numeric_limits<unsigned int>::max() and the
	// * default value of fixed_body_discriminator is max (unsigned int) / 2.
	rrt_.dofSize = nb_of_joints_;


	// rbdl이 상체 모든 걸 다 받아버리면 말단 장치 위치 계산할때 모든 관절의 위치를 알고 있어야함.
	end_effector_id_ = rbdl_model_.GetBodyId((config_.chain_end).c_str());

	if (rbdl_model_.IsFixedBodyId(end_effector_id_))
	{
		end_effector_com_ = rbdl_model_.mFixedBodies[end_effector_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
	}
	else
	{
		end_effector_com_ = rbdl_model_.mBodies[end_effector_id_].mCenterOfMass;
	}
	// end_effector_com_ : 0.0, 0.0, 0.0 for robocare

	//cout << "end_effector_com_" << end_effector_com_.transpose() << endl;


}
// void ArmPlanner::initializeData(arm_motion_planner::plan_arm_motion::Request &req) {

// 	// joint limit 
// 	assert(nb_of_joints_ == config_.joint_limit_lower.size());
// 	assert(nb_of_joints_ == config_.joint_limit_upper.size());

// 	joint_limit_.lower_.resize(nb_of_joints_);
// 	joint_limit_.lower_rad_.resize(nb_of_joints_);
// 	joint_limit_.upper_.resize(nb_of_joints_);
// 	joint_limit_.upper_rad_.resize(nb_of_joints_);

// 	for (std::vector<int>::size_type i=0;i<config_.joint_limit_lower.size();i++){
// 		joint_limit_.lower_(i) = config_.joint_limit_lower[i];
// 		joint_limit_.upper_(i) = config_.joint_limit_upper[i];
// 	}
	
// 	joint_limit_.lower_rad_ = joint_limit_.lower_ / 180.0*M_PI;
// 	joint_limit_.upper_rad_ = joint_limit_.upper_ / 180.0*M_PI;

// 	// current joint position
// 	joint_state_.qInit_.resize(req.current_joint_state.position.size());
// 	for (int i =0;i<req.current_joint_state.position.size();i++)
// 		joint_state_.qInit_(i) = req.current_joint_state.position[i];

// 	output_trajectory_.joint_names = req.current_joint_state.name;

// 	// current base position
// 	mobile_pose_.qInit_.resize(3);
// 	mobile_pose_.qInit_(0) = req.current_mobile_state.x;
// 	mobile_pose_.qInit_(1) = req.current_mobile_state.y;
// 	mobile_pose_.qInit_(2) = req.current_mobile_state.theta;
// 	mobile_pose_.rotInit_ = rotateZaxis(req.current_mobile_state.theta);

// 	// initial pose &  target pose in Global frame
// 	init_pose_.translation().head(2) = mobile_pose_.qInit_.head(2);
// 	init_pose_.translation()(2) = 0.0;
// 	//init_pose_.translation() += mobile_pose_.rotInit_ * base_frame_pos_;
// 	init_pose_.translation() += mobile_pose_.rotInit_ * CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
// 	init_pose_.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();

// 	//cout << "init_pose_" <<init_pose_.translation().transpose() << endl;

// 	target_pose_.translation()(0) = req.target_ee_pose.position.x;
// 	target_pose_.translation()(1) = req.target_ee_pose.position.y;
// 	target_pose_.translation()(2) = req.target_ee_pose.position.z;
// 	Eigen::Quaterniond quat(req.target_ee_pose.orientation.w, req.target_ee_pose.orientation.x, req.target_ee_pose.orientation.y, req.target_ee_pose.orientation.z);
// 	target_pose_.linear() = quat.toRotationMatrix();
// 	//cout << "target_pose_.linear()" << target_pose_.linear() << endl;

// 	Vector3d from_init_to_goal_in_local = mobile_pose_.rotInit_.transpose()*(init_pose_.translation() - target_pose_.translation());

// 	//cout << from_init_to_goal_in_local.transpose() << endl;

// 	// Obstacles 
// 	rrt_.box_num_obs = req.Obstacles3D.size();
// 	for (int i=0;i<rrt_.box_num_obs;i++)
// 	{
// 		rrt_.Box_obs[i].fAxis = Vector3d(req.Obstacles3D[i].Box_dimension.x, req.Obstacles3D[i].Box_dimension.y, req.Obstacles3D[i].Box_dimension.z);
// 		rrt_.Box_obs[i].vAxis[0] = mobile_pose_.rotInit_.transpose() * Vector3d(1, 0, 0);
// 		rrt_.Box_obs[i].vAxis[1] = mobile_pose_.rotInit_.transpose() * Vector3d(0, 1, 0);
// 		rrt_.Box_obs[i].vAxis[2] = mobile_pose_.rotInit_.transpose() * Vector3d(0, 0, 1);
// 		rrt_.Box_obs[i].vPos =  mobile_pose_.rotInit_.transpose()* Vector3d(req.Obstacles3D[i].Box_pose.position.x - mobile_pose_.qInit_(0), req.Obstacles3D[i].Box_pose.position.y - mobile_pose_.qInit_(1), req.Obstacles3D[i].Box_pose.position.z); // axis center pos
// 	}

// 	// link data (link dimension)
// 	rrt_.box_num_link = config_.link_name.size();
// 	for (int i=0;i<rrt_.box_num_link;i++){
// 		Vector3d link_dim;
// 		for (int j=0;j<config_.link_dimension[i].size();j++){
// 			link_dim(j) =  config_.link_dimension[i][j];
// 			rrt_.Box_link[i].vCenter(j) = config_.link_position[i][j];
// 		}

// 		rrt_.Box_link[i].vRot = rotateXaxis(config_.link_orientation[i][0])*rotateYaxis(config_.link_orientation[i][1])*rotateZaxis(config_.link_orientation[i][2]);
// 		rrt_.Box_link[i].fAxis = link_dim;
// 	}

// 	// Constraint bound
// 	constrain_pose_ = req.Pose_bound.constrain_pose.data;
// 	rrt_.C.resize(6,2);
// 	for (int i = 0; i < 3; i++)
// 	{
// 		if (from_init_to_goal_in_local(i) > 0)
// 		{
// 			rrt_.C(i, 0) = from_init_to_goal_in_local(i) + 0.05;
// 			rrt_.C(i, 1) = -0.05;
// 		}
// 		else
// 		{
// 			rrt_.C(i, 0) = 0.05;
// 			rrt_.C(i, 1) = from_init_to_goal_in_local(i) - 0.05;
// 		}
// 	}

// 	// rrt_.C(0,0) = req.Pose_bound.position_bound_upper.x;
// 	// rrt_.C(1,0) = req.Pose_bound.position_bound_upper.y;
// 	// rrt_.C(2,0) = req.Pose_bound.position_bound_upper.z;

// 	// rrt_.C(0,1) = req.Pose_bound.position_bound_lower.x;
// 	// rrt_.C(1,1) = req.Pose_bound.position_bound_lower.y;
// 	// rrt_.C(2,1) = req.Pose_bound.position_bound_lower.z;

// 	rrt_.C(3,0) = req.Pose_bound.orientation_bound_upper.x;
// 	rrt_.C(4,0) = req.Pose_bound.orientation_bound_upper.y;
// 	rrt_.C(5,0) = req.Pose_bound.orientation_bound_upper.z;

// 	rrt_.C(3,1) = req.Pose_bound.orientation_bound_lower.x;
// 	rrt_.C(4,1) = req.Pose_bound.orientation_bound_lower.y;
// 	rrt_.C(5,1) = req.Pose_bound.orientation_bound_lower.z;


// 	// Trajectory library 
// 	interpolate_path_ = req.interpolate_path.data;

// 	maxVelocity.resize(nb_of_joints_);
// 	maxVelocity.setZero();
// 	maxAcceleration.resize(nb_of_joints_);
// 	maxAcceleration.setZero();
// 	for (size_t i = 0; i < nb_of_joints_; i++)
// 	{
// 		maxAcceleration(i) = 10.0;
// 		maxVelocity(i) = 10.0;
// 	}
// 	wayPoints.clear();
// 	playTime_ = 0.0;

// }

trajectory_msgs::JointTrajectory ArmPlanner::compute() {
	// get Body Id of Link
	body_id_collision_.clear();
	body_com_position_.clear();
	for (std::vector<int>::size_type i = 0; i < config_.link_dimension.size(); i++)
	{
		body_id_collision_.push_back(rbdl_model_.GetBodyId(config_.link_name[i].c_str()));
		body_com_position_.push_back(rbdl_model_.mBodies[rbdl_model_.GetBodyId(config_.link_name[i].c_str())].mCenterOfMass);
	};

	// push back end_effector
	body_id_collision_.push_back(end_effector_id_);
	body_com_position_.push_back(end_effector_com_);
	
	// Write the model data
	rrt_model_.model_ = rbdl_model_;
	rrt_model_.body_id_vec.clear();
	rrt_model_.body_id_vec.assign(body_id_collision_.begin(), body_id_collision_.end());
	rrt_model_.body_com_pos.clear();
	rrt_model_.body_com_pos.assign(body_com_position_.begin(), body_com_position_.end());

	// for (int i=0;i<5;i++){
	// cout << CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, body_id_collision_[i], body_com_position_[i], true).transpose()<< endl;
	// }
	// Calculate IK solution
	std::vector<VectorXd> q_goal_set;
	int nb_of_sols = 0 ;
	for (int i = 0; i < 10; i++)
	{
		if (solveIK(target_pose_, rrt_model_))
		{
			q_goal_set.push_back(joint_state_.qGoal_);
			nb_of_sols ++;	
		}
	}

	if (nb_of_sols == 0)
	{
		ROS_INFO_STREAM("IK solutions does not exist!!");
		output_trajectory_.points.clear();
		return output_trajectory_;
	}

	// Plan the trajectory
	if(constrain_pose_)
	{
		rrt_.refer_pos = target_pose_.translation(); 
		rrt_.refer_rot = target_pose_.linear();

		rrt_.local_pos.head(2) = mobile_pose_.qInit_.head(2);
		rrt_.local_pos(2) = 0.0;
		rrt_.local_rot = mobile_pose_.rotInit_;

		VectorXd q_tra(7);
		Vector3d ee_pos;
		//if (CRRT_planning(joint_state_.qInit_, joint_state_.qGoal_, joint_target2_))

		if (solveCRRTwithMultipleGoals(joint_state_.qInit_, q_goal_set, joint_target2_))
		{
			if (!interpolate_path_)
			{
				output_trajectory_.points.clear();
				for (int i = 0; i < joint_target2_.rows(); i++)
				{
					wayPoints.push_back(joint_target2_.row(i));
				}
				trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
				//	trajectory_generator_->outputPhasePlaneTrajectory();
				duration_ = trajectory_generator_->getDuration();
				//cout <<"duration" << duration_ << endl;
				while (playTime_ / 10.0 < duration_)
				{
					trajectory_point_.positions.clear();
					trajectory_point_.velocities.clear();

					for (int i = 0; i < nb_of_joints_; i++)
					{
						trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
						trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					}
					trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);

					output_trajectory_.points.push_back(trajectory_point_);

					ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, trajectory_generator_->getPosition(playTime_ / 10.0) / 180.0 * M_PI, end_effector_id_, end_effector_com_, true);
					//cout << ee_pos.transpose() << endl;
					playTime_++;
				}
			}
			else
			{
				output_trajectory_.points.clear();

				for (int i = 0; i < joint_target2_.rows(); i++)
				{
					q_tra = joint_target2_.row(i) / 180.0 * M_PI;
					ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, q_tra, end_effector_id_, end_effector_com_, true);
					//cout << ee_pos.transpose() << endl;
					//cout << joint_target2_.row(i)/180.0*M_PI << endl;
					//cout << "\n"
					//	 << endl;
					fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", joint_target2_(i, 0), joint_target2_(i, 1), joint_target2_(i, 2), joint_target2_(i, 3), joint_target2_(i, 4), joint_target2_(i, 5), joint_target2_(i, 6), ee_pos(0), ee_pos(1), ee_pos(2));
					trajectory_point_.positions.clear();
					for (int j = 0; j < joint_target2_.cols(); j++)
						trajectory_point_.positions.push_back(joint_target2_(i, j));

					output_trajectory_.points.push_back(trajectory_point_);
				}
			}
		}
		else
		{
			output_trajectory_.points.clear();
		}
	}
	else
	{
		if (solveRRTwithMultipleGoals(joint_state_.qInit_, q_goal_set, joint_target_))
		{
			if (!interpolate_path_)
			{
				output_trajectory_.points.clear();
				for (int i = 0; i < joint_target_.rows(); i++)
				{
					wayPoints.push_back(joint_target_.row(i));
				}
				trajectory_generator_ = new Trajectory(Path(wayPoints, 0.1), maxVelocity, maxAcceleration);
				//	trajectory_generator_->outputPhasePlaneTrajectory();
				duration_ = trajectory_generator_->getDuration();
				//cout <<"duration" << duration_ << endl;
				while (playTime_ / 10.0 < duration_)
				{
					trajectory_point_.positions.clear();
					trajectory_point_.velocities.clear();

					for (int i = 0; i < nb_of_joints_; i++)
					{
						trajectory_point_.positions.push_back(trajectory_generator_->getPosition(playTime_ / 10.0)[i]);
						trajectory_point_.velocities.push_back(trajectory_generator_->getVelocity(playTime_ / 10.0)[i]);
					}
					trajectory_point_.time_from_start = ros::Duration(playTime_ / 10.0);

					output_trajectory_.points.push_back(trajectory_point_);

					playTime_++;
				}
			}
			else
			{
				output_trajectory_.points.clear();

				for (int i = 0; i < joint_target_.rows(); i++)
				{
					//q_tra = joint_target_.row(i) / 180.0 * M_PI;
					// ee_pos = CalcBodyToBaseCoordinates(rbdl_model_, q_tra, end_effector_id_, end_effector_com_, true);
					// cout << ee_pos.transpose() << endl;
					cout <<"Path\t" << i << "\t" << "rad:" << joint_target_.row(i)/180.0*M_PI << endl;
					cout << "Path\t" << i << "\t" << "angle:" << joint_target_.row(i) << endl;
					//fprintf(fp1, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t \n", joint_target2_(i, 0), joint_target2_(i, 1), joint_target2_(i, 2), joint_target2_(i, 3), joint_target2_(i, 4), joint_target2_(i, 5), joint_target2_(i, 6), ee_pos(0), ee_pos(1), ee_pos(2));
					trajectory_point_.positions.clear();
					for (int j = 0; j < joint_target_.cols(); j++)
						trajectory_point_.positions.push_back(joint_target_(i, j));

					output_trajectory_.points.push_back(trajectory_point_);
				}
			}

		}
		else
		{
			output_trajectory_.points.clear();

		}
	}

	return output_trajectory_;
}


bool ArmPlanner::solveIK(Transform3d pose, Robotmodel& model) // from panda_arm.xacro
{
	double eps = 1e-7;
	double num_samples = 100;
	double timeout = 0.005;
	std::string chain_start;
	std::string chain_end;

	chain_start = config_.chain_start;
	chain_end = config_.chain_end;
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout, eps);
	
	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL chain found");
		return false;
	}

	valid = tracik_solver.getKDLLimits(ll, ul);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL joint limits found");
		return false;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	// Set up KDL IK
	KDL::ChainFkSolverPos_recursive fk_solver(chain);									  // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(chain);										  // PseudoInverse vel solver
	KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
	// 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(chain.getNrOfJoints());

	// Create desired number of valid, random joint configurations
	std::vector<KDL::JntArray> JointList;
	KDL::JntArray q(chain.getNrOfJoints());

	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;
	KDL::Frame end_effector_pose;

	Matrix4d target_from_base;
	Matrix4d target_global;
	Matrix4d base_global;
	base_global.setIdentity();
	target_from_base.setIdentity();
	target_global.setIdentity();

	base_global.block(0,0,3,3) = rotateZaxis(mobile_pose_.qInit_(2));
	base_global(0, 3) = mobile_pose_.qInit_(0);
	base_global(1, 3) = mobile_pose_.qInit_(1);
	base_global(2, 3) = 0.0;

	target_global.block(0, 0, 3, 3) = pose.linear();
	target_global.block(0, 3, 3, 1) = pose.translation();

	target_from_base = base_global.inverse() * target_global;

	//cout << "target_global" << target_global << endl;
	//cout << "base_global" << base_global << endl;

	for (int i = 0; i < 3; i++)
	{
		end_effector_pose.p(i) = target_from_base(i,3);
	}
	//cout << "waist to desired ee in local : " << target_from_base.block(0, 3, 3, 1).transpose() << endl;

	KDL::Rotation A;
	A.data[0] = target_from_base(0, 0);
	A.data[1] = target_from_base(0, 1);
	A.data[2] = target_from_base(0, 2);
	A.data[3] = target_from_base(1, 0);
	A.data[4] = target_from_base(1, 1);
	A.data[5] = target_from_base(1, 2);
	A.data[6] = target_from_base(2, 0);
	A.data[7] = target_from_base(2, 1);
	A.data[8] = target_from_base(2, 2);
	end_effector_pose.M = A;

	int rc;

	double total_time = 0;
	uint success = 0;
	bool solved = true;

	while(true) //for (uint i = 0; i < num_samples; i++)
	{
		if(--num_samples == 0){
			solved = false;
			break;
		}

		std::vector<double> R;
		for (int i = 0; i < chain.getNrOfJoints(); i++)
		{
			double jointrange = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i); // angle
			double r = ((double)rand() / (double)RAND_MAX) * jointrange;
			R.push_back(joint_limit_.lower_rad_(i) + r);
		}

		for (size_t j = 0; j < nominal.data.size(); j++)
		{
			nominal(j) = R[j];
		}

		//cout <<"iteration?" << endl;
		double elapsed = 0;
		start_time = boost::posix_time::microsec_clock::local_time();
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
		// int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());

		std::vector<double> config;
		config.clear();
		if (rc >= 0)
		{
			//ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);

			for (int i = 0; i < nb_of_joints_; i++)
			{
				config.push_back(result.data(i) * 180 / M_PI);
			}
			// joint_state_.qGoal_ = result.data;
			// solved = true;
			// break;
			if (!rrt_.checkExternalCollision(model, config) && !rrt_.checkSelfCollision(model, config) )
			{

				ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
				joint_state_.qGoal_ = result.data;
				solved = true;
				break;
			}
			else
			{
				continue;
			}
		}
		else
		{
			continue;
		}
	}
	return solved;
}
bool ArmPlanner::setupRRT(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{
		ofstream outFile2("path_result2.txt", ios::out); // "writing"
		// path_result -> Smooth -> path_result2
		ifstream inFile("path_result.txt"); // "reading"
		rrt_.smoothPath(outFile2, inFile);

		outFile2.close();
		MatrixXd joint_temp(100, nb_of_joints_);

		ifstream inFile2("path_result2.txt"); // "reading"
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[1000];
		while (!inFile2.eof())
		{ // eof : end of file
			inFile2.getline(inputString, 1000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		//cout << "trajectory size" << size << endl;
		inFile2.close();
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}
}
bool ArmPlanner::setupCRRT(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoal = q_goal;

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[50000];
		while (!inFile3.eof())
		{
			inFile3.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile3.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}



bool ArmPlanner::solveRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"

	if (rrt_.solveRRT(rrt_model_, outFile))
	{

		outFile.close();
		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile2("path_result.txt"); // "reading"
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[1000];
		while (!inFile2.eof())
		{
			inFile2.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile2.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}

bool ArmPlanner::solveCRRTwithMultipleGoals(VectorXd q_start, std::vector<VectorXd> q_goal_set, MatrixXd &joint_target)
{
	rrt_.lower_limit = joint_limit_.lower_;
	rrt_.upper_limit = joint_limit_.upper_;

	rrt_.qinit = q_start;
	rrt_.qgoals.assign(q_goal_set.begin(), q_goal_set.end());

	ofstream outFile3("path_result3.txt", ios::out);

	if (rrt_.solveCRRT(rrt_model_, outFile3))
	{
		outFile3.close();

		//cout << "111"<<endl;
		MatrixXd joint_temp(5000, nb_of_joints_);

		ifstream inFile3("path_result3.txt");
		int size = 0;
		std::vector<std::string> parameters;
		char inputString[50000];
		while (!inFile3.eof())
		{
			inFile3.getline(inputString, 50000);
			boost::split(parameters, inputString, boost::is_any_of(","));
			if (parameters.size() == nb_of_joints_)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		inFile3.close();
		//cout << "size" << size << endl;
		joint_target = joint_temp.topLeftCorner(size, nb_of_joints_);
		
		return true;
	}
	else
	{
		cout << "Time out!!" << endl;
		return false;
	}

}