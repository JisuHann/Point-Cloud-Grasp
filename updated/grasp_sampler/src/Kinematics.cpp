#include "../include/grasp_sampler/Kinematics.h"

kinematics_sovler::kinematics_sovler(YAMLConfig &config):
config_(config)
{
    initModel();

	//////// motion planenr ///////////////////////////////////

	///////// parameter initialize ////////////////////////////


	///////// memory allocate /////////////////////////////////

	reachability_cloud_ = pcl::PointCloud <pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void kinematics_sovler::initModel()
{
    std::string urdf_absolute_path;
	std::string mod_url = config_.urdf_path;
	if (config_.urdf_path.find("package://") == 0)
	{
		mod_url.erase(0, strlen("package://"));
		size_t pos = mod_url.find("/");
		if (pos == std::string::npos)
		{
			std::cout << "Could not parse package:// format into file:// format" << std::endl;
		}
		std::string package = mod_url.substr(0, pos);
		mod_url.erase(0, pos);
		std::string package_path = ros::package::getPath(package);

		if (package_path.empty())
		{
			std::cout << "Package does not exist" << std::endl;
		}

		urdf_absolute_path =  package_path + mod_url;
	}

	RigidBodyDynamics::Addons::URDFReadFromFile(urdf_absolute_path.c_str(), &rbdl_model_, false, false);

	// Serial Robot DOF = nb_of_joint
	nb_of_joints_ = rbdl_model_.q_size;
	rrt_.dofSize = nb_of_joints_;


	end_effector_id_ = rbdl_model_.GetBodyId((config_.chain_end).c_str());
	arm_base_frame_id_ = rbdl_model_.GetBodyId((config_.chain_start).c_str());

	// std::cout<<"nb of joints  "<<nb_of_joints_<<std::endl;
	// std::cout<<"ee_frame_num  "<<(int)end_effector_id_<<"  b_Frame_num "<<(int)arm_base_frame_id_<<std::endl;

	if (rbdl_model_.IsFixedBodyId(end_effector_id_))
	{
		end_effector_com_ = rbdl_model_.mFixedBodies[end_effector_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
	}
	else
	{
		end_effector_com_ = rbdl_model_.mBodies[end_effector_id_].mCenterOfMass;
	}

	if (rbdl_model_.IsFixedBodyId(arm_base_frame_id_))
	{
		arm_base_frame_pos_ = rbdl_model_.mFixedBodies[arm_base_frame_id_ - rbdl_model_.fixed_body_discriminator].mCenterOfMass;
	}
	else
	{
		arm_base_frame_pos_ = rbdl_model_.mBodies[arm_base_frame_id_].mCenterOfMass;
	}

	std::cout << "Model Loaded" << std::endl;
}

void kinematics_sovler::initializeData(task_assembly::door_open_planner::Request &req,geometry_msgs::Pose targetPose)
{
	// joint limit 
	assert(nb_of_joints_ == config_.joint_limit_lower.size());
	assert(nb_of_joints_ == config_.joint_limit_upper.size());

	int nb_of_joint_val = req.current_arm_joint_state.position.size();

	// current joint state update
	cur_joint_val_ = KDL::JntArray(nb_of_joint_val);

	for(unsigned int jnt_num = 0 ;jnt_num < nb_of_joint_val; jnt_num++)
		cur_joint_val_(jnt_num)= req.current_arm_joint_state.position.at(jnt_num);  //in rad

	// Memory Allocate
	joint_limit_.lower_.resize(nb_of_joints_);
	joint_limit_.lower_rad_.resize(nb_of_joints_);
	joint_limit_.upper_.resize(nb_of_joints_);
	joint_limit_.upper_rad_.resize(nb_of_joints_);

	for (std::vector<int>::size_type i=0;i<config_.joint_limit_lower.size();i++){
		joint_limit_.lower_(i) = config_.joint_limit_lower[i];
		joint_limit_.upper_(i) = config_.joint_limit_upper[i];
	}

	// Deg to rad
	joint_limit_.lower_rad_ = joint_limit_.lower_ / 180.0*M_PI;
	joint_limit_.upper_rad_ = joint_limit_.upper_ / 180.0*M_PI;

	joint_state_.qInit_.resize(req.current_arm_joint_state.position.size());
	for (int i =0;i<req.current_arm_joint_state.position.size();i++)
		joint_state_.qInit_(i) = req.current_arm_joint_state.position[i];

	output_trajectory_.joint_names = req.current_arm_joint_state.name;

	// initial pose &  target pose in Global frame
	init_pose_.translation() = CalcBodyToBaseCoordinates(rbdl_model_, joint_state_.qInit_, end_effector_id_, end_effector_com_, true);
	init_pose_.linear() = CalcBodyWorldOrientation(rbdl_model_ , joint_state_.qInit_, end_effector_id_, true).transpose();

	target_pose_.translation()(0) = targetPose.position.x;
	target_pose_.translation()(1) = targetPose.position.y;
	target_pose_.translation()(2) = targetPose.position.z;
	Eigen::Quaterniond quat(targetPose.orientation.w,targetPose.orientation.x, targetPose.orientation.y, targetPose.orientation.z);
	target_pose_.linear() = quat.toRotationMatrix();


	// link data (link dimension)
	rrt_.box_num_link = config_.link_name.size();
	for (int i=0;i<rrt_.box_num_link;i++){
		Vector3d link_dim;
		for (int j=0;j<config_.link_dimension[i].size();j++){
			link_dim(j) =  config_.link_dimension[i][j];
			rrt_.Box_link[i].vCenter(j) = config_.link_position[i][j];
		}

		rrt_.Box_link[i].vRot = rotateXaxis(config_.link_orientation[i][0])*rotateYaxis(config_.link_orientation[i][1])*rotateZaxis(config_.link_orientation[i][2]);
		rrt_.Box_link[i].fAxis = link_dim;
	}


	// Trajectory library 
	interpolate_path_ = req.interpolate_path.data;

	maxVelocity.resize(nb_of_joints_);
	maxVelocity.setZero();
	maxAcceleration.resize(nb_of_joints_);
	maxAcceleration.setZero();
	for (size_t i = 0; i < nb_of_joints_; i++)
	{
		maxAcceleration(i) = 10.0;
		maxVelocity(i) = 10.0;
	}
	wayPoints.clear();
	playTime_ = 0.0;

	this->initializeIKparam(config_.chain_start, config_.chain_end, urdf_param_);
	calcReachability();
}

bool kinematics_sovler::initializeIKparam(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param)
{
	ros::NodeHandle node_handle("~");

	std::string xml_string;

	std::string urdf_xml, full_urdf_xml;
	node_handle.param("urdf_xml", urdf_xml, URDF_param);
	node_handle.searchParam(urdf_xml, full_urdf_xml);

	//ROS_DEBUG_NAMED("IK", "Reading xml file from parameter server");
	if (!node_handle.getParam(full_urdf_xml, xml_string))
	{
		std::cout<<"Could not load the xml from parameter server"<<std::endl;
		return false;
	}

	node_handle.param(full_urdf_xml, xml_string, std::string());
	IK_robot_model.initString(xml_string);

	//ROS_DEBUG_STREAM_NAMED("trac_ik", "Reading joints and links from URDF");

	if (!kdl_parser::treeFromUrdfModel(IK_robot_model, IK_tree))
		std::cout<<"Failed to extract kdl tree from xml robot description"<<std::endl;

	if (!IK_tree.getChain(base_link, tip_link, IK_chain))
		std::cout<<"Couldn't find chain"<<std::endl;

	std::vector<KDL::Segment> chain_segs = IK_chain.segments;

	urdf::JointConstSharedPtr joint;

	IK_lb.resize(IK_chain.getNrOfJoints());
	IK_ub.resize(IK_chain.getNrOfJoints());

	uint joint_num = 0;
	for (unsigned int i = 0; i < chain_segs.size(); ++i)
	{
		joint = IK_robot_model.getJoint(chain_segs[i].getJoint().getName());
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
		{
			joint_num++;
			float lower, upper;
			int hasLimits;
			if (joint->type != urdf::Joint::CONTINUOUS)
			{
				if (joint->safety)
				{
					lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
					upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
				}
				else
				{
					lower = joint->limits->lower;
					upper = joint->limits->upper;
				}
				hasLimits = 1;
			}
			else
			{
				hasLimits = 0;
			}
			if (hasLimits)
			{
				IK_lb(joint_num - 1) = lower;
				IK_ub(joint_num - 1) = upper;
			}
			else
			{
				IK_lb(joint_num - 1) = std::numeric_limits<float>::lowest();
				IK_ub(joint_num - 1) = std::numeric_limits<float>::max();
			}
			//ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));
		}
	}
}

Eigen::Matrix4f kinematics_sovler::Frame2Eigen(KDL::Frame &frame)
{
	Eigen::Matrix4f H_trans;
    H_trans.resize(4,4);
    double d[16] = {0,};
    frame.Make4x4(d);

    H_trans(0,0) = d[0];
    H_trans(0,1) = d[1];
    H_trans(0,2) = d[2];
    H_trans(0,3) = d[3];
    H_trans(1,0) = d[4];
    H_trans(1,1) = d[5];
    H_trans(1,2) = d[6];
    H_trans(1,3) = d[7];
    H_trans(2,0) = d[8];
    H_trans(2,1) = d[9];
    H_trans(2,2) = d[10];
    H_trans(2,3) = d[11];
    H_trans(3,0) = d[12];
    H_trans(3,1) = d[13];
    H_trans(3,2) = d[14];
    H_trans(3,3) = d[15];

    return H_trans;
}

Eigen::Matrix4f kinematics_sovler::solveFK(KDL::JntArray _joint_val)
{
    //robot_joint_val_= cur_joint_val_;
	robot_joint_val_ = _joint_val;

    KDL::ChainFkSolverPos_recursive Fksolver = KDL::ChainFkSolverPos_recursive(IK_chain);

    if(Fksolver.JntToCart(robot_joint_val_,T_BC_Frame_) < 0)
        std::cerr<<"Fk about robot Model Failed!!!!"<<std::endl;

    // T_BO = T_BC * T_CO ---> External Params
	return Frame2Eigen(T_BC_Frame_);
}

bool kinematics_sovler::solveIK(Transform3d _target_ee_pose, Robotmodel& model)
{
	/////////////////// check reachability /////////////////////////////////////////

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr copy_reachability_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(*reachability_cloud_, *copy_reachability_cloud_);
	// for test
	pcl::PointXYZRGB point;
	point.x = _target_ee_pose.translation().x();
	point.y = _target_ee_pose.translation().y();
	point.z = _target_ee_pose.translation().z();
	point.r = 255;	
	point.g = 0;
	point.b = 0;
	copy_reachability_cloud_->push_back(point);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  	tree->setInputCloud(copy_reachability_cloud_);

	std::vector<pcl::PointIndices> cluster_indicese; 
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setInputCloud(copy_reachability_cloud_);       // 입력   
	ec.setClusterTolerance(0.05);  // 5cm  
	ec.setMinClusterSize(1);     // 최소 포인트 수 
	ec.setMaxClusterSize(200000);   // 최대 포인트 수
	ec.setSearchMethod(tree);      // 위에서 정의한 탐색 방법 지정 
	ec.extract(cluster_indicese);   // 군집화 적용 

	if(cluster_indicese.size() > nb_of_clusters_)
	{
		std::cout << "nb of  clusters :"<<nb_of_clusters_<<std::endl;
		std::cout<<"reachability error ocurred !!!"<<std::endl;
		return false;
	}

	////////////////////////////////////////////////////////////////////////////////

	//track - IK params
	double eps = 1e-7;
	double num_samples = 100;
	double timeout = 0.005;

	std::string chain_start = config_.chain_start;
	std::string chain_end= config_.chain_end;

	KDL::Frame end_effector_pose;

	/////////////// setup track-ik //////////////////

	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout, eps);
	bool valid = tracik_solver.getKDLChain(IK_chain);

	if (!valid)
	{
		std::cout<<"There was no valid KDL chain found"<<std::endl;
		return false;
	}

	valid = tracik_solver.getKDLLimits(IK_lb,IK_ub);

	if (!valid)
	{
		std::cout<<"There were no valid KDL joint limits found"<<std::endl;
		return false;
	}

	//////// setup target configuration ////////////////////////////

	Eigen::Matrix4d target_global;
	target_global.block(0, 0, 3, 3) = _target_ee_pose.linear();
	target_global.block(0, 3, 3, 1) = _target_ee_pose.translation();

	for (int i = 0; i < 3; i++)
	{
		end_effector_pose.p(i) = target_global(i,3);
	}

	KDL::Rotation rot;
	rot.data[0] = target_global(0, 0);
	rot.data[1] = target_global(0, 1);
	rot.data[2] = target_global(0, 2);
	rot.data[3] = target_global(1, 0);
	rot.data[4] = target_global(1, 1);
	rot.data[5] = target_global(1, 2);
	rot.data[6] = target_global(2, 0);
	rot.data[7] = target_global(2, 1);
	rot.data[8] = target_global(2, 2);
	end_effector_pose.M = rot;

	KDL::JntArray nominal(IK_chain.getNrOfJoints());

	int rc;
	double total_time = 0;
	uint success = 0;
	bool solved = true;

	nominal = cur_joint_val_;  //---> why?? 

	// std::cout<<"nominal :"<<nominal.data(6)<<std::endl;
	// std::cout<<"cur_joint_val_ :"<<cur_joint_val_.data(6)<<std::endl;

	std::cout<<"solving IK..."<<std::endl;

	// rc = tracik_solver.CartToJnt(nominal, end_effector_pose, IK_result_);

	// if (rc <= 0)
	// {
	// 	std::cout<<"IK solution Not Exist!!!"<<std::endl;
	// 	return false;
	// }

	// ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
	// std::cout<<IK_result_.data.transpose()<<std::endl;
	// return true;

	while(true) //for (uint i = 0; i < num_samples; i++)
	{
		if(--num_samples == 0){
			solved = false;
			break;
		}

		std::vector<double> R;
		for (int i = 0; i < IK_chain.getNrOfJoints(); i++)
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
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, IK_result_);
		// int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
		
		std::vector<double> config;	
		config.clear();

		if (rc >= 0)
		{
			//ROS_INFO_STREAM("IK solution is : " << result.data.transpose() * 180 / M_PI);
			for (int i = 0; i < nb_of_joints_; i++)
			{
				config.push_back(IK_result_.data(i) * 180 / M_PI);
			}
			
			if (!rrt_.checkSelfCollision(model, config) )
			{

				ROS_INFO_STREAM("IK solution is : " << IK_result_.data.transpose() * 180 / M_PI);
				joint_state_.qGoal_ = IK_result_.data;
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

void kinematics_sovler::calcReachability()
{
	std::cout<<"calculating reachability....."<<std::endl;
	std::cout<<"sampling manipualtor configuration....."<<std::endl;

	// Eigen::Vector3d link_com_pose;

	// for(unsigned int joint_num = 1 ; joint_num < 8 ; joint_num++)
	// {

	// }
	// Sphere 
	// 1. calc L2 norm
	// 2. if robot base to grasp pose Euclidean dist > L2 norm IK not Exist  

	// reachability imagine 
	// 1. sampling points from random joint variable
	// 2. make 3d polygon from points 
	// 3. show grasp pose is in 3d polygon area 
	
	/////// sampling random configuration/////////////////////////////////////////////////////////
	
	KDL::JntArray nominal(IK_chain.getNrOfJoints());

	int sampling_cnt = 0;
	while(sampling_cnt < nb_of_sampling_points_)
	{
		std::vector<double> R;
		for (int i = 0; i < IK_chain.getNrOfJoints(); i++)		//Sampling Random 시작 위치
		{
			double jointrange = joint_limit_.upper_rad_(i) - joint_limit_.lower_rad_(i); // angle
			double r = ((double)rand() / (double)RAND_MAX) * jointrange;			//normalized Random value
			R.push_back(joint_limit_.lower_rad_(i) + r);
		}

		for (size_t j = 0; j < nominal.data.size(); j++)
		{
			nominal(j) = R[j];
			//std::cout<<"joint val  "<<R[j]<<std::endl;
		}

		Eigen::Matrix4f FK_pose = solveFK(nominal);

		//std::cout<<FK_pose<<std::endl;

		pcl::PointXYZRGB point;
		point.x = FK_pose(0,3);
		point.y = FK_pose(1,3);
		point.z = FK_pose(2,3);
		point.r = 255;	
		point.g = 0;
		point.b = 0;

		reachability_cloud_->push_back(point);
		sampling_cnt++;	

	}
	std::cout<<"sampling done !!!"<<std::endl;
	/////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////// Clustering ///////////////////////////////////////////////////////
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  	tree->setInputCloud(reachability_cloud_);

	std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
	// 군집화 오브젝트 생성  
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setInputCloud(reachability_cloud_);       // 입력   
	ec.setClusterTolerance(0.05);  // 5cm  
	ec.setMinClusterSize(1);     // 최소 포인트 수 
	ec.setMaxClusterSize(200000);   // 최대 포인트 수
	ec.setSearchMethod(tree);      // 위에서 정의한 탐색 방법 지정 
	ec.extract(cluster_indices);   // 군집화 적용 
	nb_of_clusters_ = cluster_indices.size();

	std::cout << "nb of  clusters :"<<nb_of_clusters_<<std::endl;

	// for test
	// pcl::PointXYZRGB point;
	// point.x = 1.6;
	// point.y = 1.6;
	// point.z = 1.6;
	// point.r = 255;	
	// point.g = 0;
	// point.b = 0;
	// reachability_cloud_->push_back(point);

	// std::vector<pcl::PointIndices> cluster_indicese; 
	// ec.setInputCloud(reachability_cloud_);       // 입력   
	// ec.setClusterTolerance(0.05);  // 5cm  
	// ec.setMinClusterSize(1);     // 최소 포인트 수 
	// ec.setMaxClusterSize(200000);   // 최대 포인트 수
	// ec.setSearchMethod(tree);      // 위에서 정의한 탐색 방법 지정 
	// ec.extract(cluster_indicese);   // 군집화 적용 
	// nb_of_clusters_ = cluster_indicese.size();

	//std::cout << "nb of  clusters  after :"<<nb_of_clusters_<<std::endl;

	if(reachability_cloud_->size() > 0)
    {
        std::string save_path = "/home/dyros/tmp_ws/src/grasp_sampler/MODEL/reachability_cloud_";
        pcl::io::savePCDFileASCII (save_path+".pcd", *reachability_cloud_);
    }
	std::cout << "reachability claculation end"<<std::endl;
}

trajectory_msgs::JointTrajectory kinematics_sovler::generateTrajectory()
{	
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

	std::vector<VectorXd> q_goal_set;
	int nb_of_sols = 0 ;
	for (int i = 0; i < 10; i++)
	{
		if (solveIK(target_pose_,rrt_model_) )
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

	return output_trajectory_;
}

bool kinematics_sovler::setupRRT(Eigen::VectorXd q_start, Eigen::VectorXd q_goal, Eigen::MatrixXd& joint_target)
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

bool kinematics_sovler::solveRRTwithMultipleGoals(Eigen::VectorXd q_start, std::vector<Eigen::VectorXd> q_goal_set, Eigen::MatrixXd& joint_target)
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