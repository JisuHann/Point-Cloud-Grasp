#include "../include/grasp_sampler/VrepBridge.h"

sim_controller_interface::sim_controller_interface(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    is_first_run = true;
    tick = 0;
    sim_step_done_ = false;
    sim_time_ = 0.0f;
    q_.resize(TOTAL_DOF);
    q_.setZero();
    q_dot_.resize(TOTAL_DOF);
    q_dot_.setZero();
    desired_q_.resize(TOTAL_DOF);
    desired_q_.setZero();
    target_q_.resize(TOTAL_DOF);
    target_q_.setZero();
    joint_cmd_.name.resize(TOTAL_DOF);
    joint_cmd_.position.resize(TOTAL_DOF);
    goal_q_err_.resize(TOTAL_DOF);
    goal_q_err_.setZero();

    // set initial joint position
    init_q_.resize(TOTAL_DOF);
    init_q_.setZero();

    for(size_t i=0; i<TOTAL_DOF; i++)
    {
       joint_cmd_.name[i]= JOINT_NAME[i];
    }

    // pulblisher
    vrep_sim_start_pub_ = _nh.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub_ = _nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub_ = _nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub_ = _nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);
    vrep_joint_set_pub_ = _nh.advertise<sensor_msgs::JointState>("/panda/joint_set", 1);
    
    // point cloud pub
    vrep_sim_pointcloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/k4a/depth_registered/points",1);

    // subscriber
    vrep_joint_state_sub_ = _nh.subscribe("/panda/joint_states", 100, &sim_controller_interface::joint_cb, this);
    vrep_sim_step_done_sub_ = _nh.subscribe("/simulationStepDone", 100, &sim_controller_interface::sim_step_done_cb, this);
    vrep_sim_time_sub_ = _nh.subscribe("/simulationTime",100,&sim_controller_interface::sim_time_cb,this);
    vrep_sim_status_sub_ = _nh.subscribe("/simulationState",100,&sim_controller_interface::sim_status_cb,this);

    // image sub from vrep scence
    vrep_rgb_sub = _nh.subscribe("/k4a/rgb/image_rect_color",1,&sim_controller_interface::sim_image_cb,this);
    vrep_depth_sub = _nh.subscribe("/vrep/depth",1,&sim_controller_interface::sim_create_points,this);
}

  void sim_controller_interface::set_desired_q(std::vector<float> dq)
  {
    // set desired goal joint position
    if(dq.size() == TOTAL_DOF)
    {
      for(size_t i=0;i<TOTAL_DOF; i++)
      {
        desired_q_(i) = deg2rad(dq.at(i));
      }
    }
    else
    {
      for(size_t i=0;i<TOTAL_DOF; i++)
      {
        desired_q_(i) = deg2rad(-45);
      }
        desired_q_(5) = deg2rad(45);
        desired_q_(6) = deg2rad(45);
        desired_q_(7) = deg2rad(45);
    }
  }

void sim_controller_interface::set_exec_time(float t)
{
    exec_time_ = t; 
}

void sim_controller_interface::joint_cb(const sensor_msgs::JointStateConstPtr& msg)
{

    if(msg->name.size() == TOTAL_DOF)
    {
        for(size_t i=0; i< msg->name.size(); i++)
        {
        q_[i] = msg->position[i];
        q_dot_[i] = msg->velocity[i];        
        }
    }
    else
    {
        ROS_INFO("Controller's total Dof and JointStates from VREP is not same size!!");
    }
}

void sim_controller_interface::sim_status_cb(const std_msgs::Int32ConstPtr& msg)
{
    vrep_sim_status = msg->data;
}

void sim_controller_interface::read_vrep()
{
    ros::spinOnce();
}

void sim_controller_interface::compute()
{
    // first example of panda joint controller
    if(is_first_run)
    {
        start_time = tick*SIM_DT;
        final_time = start_time + exec_time_;
        is_first_run = false;
    }
    current_time = tick*SIM_DT;
    for(size_t i=0; i<TOTAL_DOF; i++)
    {
        target_q_(i) = cubic(current_time, start_time, final_time, init_q_(i), desired_q_(i), 0.0,0.0);
    }

}
void sim_controller_interface::write_vrep()
{

    for(size_t i=0;i<TOTAL_DOF;i++) {
        joint_cmd_.position[i] = target_q_(i);
        goal_q_err_(i) = abs(desired_q_(i) - q_(i));
    } 
    vrep_joint_set_pub_.publish(joint_cmd_);
    vrepStepTrigger();
}

void sim_controller_interface::wait()
{
    while(ros::ok() && !sim_step_done_)
    {
        ros::spinOnce();
    }
    sim_step_done_ = false;
    rate_.sleep();
}

void sim_controller_interface::sim_time_cb(const std_msgs::Float32ConstPtr& msg)
{
    sim_time_ = msg->data;
    tick = (sim_time_*100)/(SIM_DT*100);    
}

void sim_controller_interface::sim_step_done_cb(const std_msgs::BoolConstPtr &msg)
{
    sim_step_done_ = msg->data;
}

void sim_controller_interface::vrepStart()
{
    ROS_INFO("Starting V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_start_pub_.publish(msg);
}

void sim_controller_interface::vrepStop()
{
    ROS_INFO("Stopping V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_stop_pub_.publish(msg);
}

void sim_controller_interface::vrepStepTrigger()
{
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_step_trigger_pub_.publish(msg);
}

void sim_controller_interface::vrepEnableSyncMode()
{
    ROS_INFO("Sync Mode On");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_enable_syncmode_pub_.publish(msg);
}


void sim_controller_interface::sim_image_cb(const sensor_msgs::ImageConstPtr& img)
{
  rgb_img_ = cv_bridge::toCvShare(img, "bgr8")->image;
} 

void sim_controller_interface::sim_create_points(const std_msgs::Float32MultiArrayConstPtr& depth_img)
{
    std::vector<float> depth_raw = depth_img->data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = 480;
    cloud->height = 640;
    cloud->header.frame_id = "camera_base";
    cloud->is_dense = false;
    cloud->points.resize(cloud->width*cloud->height);
    unsigned int u_res = 480;
    unsigned int v_res = 640;
    unsigned int datalen = u_res * v_res;
    float scale = (3.5 - 0.01) / 1.0;
    std::vector<float> x_scale, y_scale;
    float f = (std::max(u_res, v_res) / 2) / tan(57.0  / 2); 
    
    if( rgb_img_.size().area() != 0)
    {
        for (int j = 0; j < u_res; j++)
        {
            float y = (j - u_res / 2.0);
            for(int i = 0; i < v_res; i++)
            {
                int k = j * v_res + i;
                float x = -(i - v_res / 2.0);
                x_scale.push_back(x / f);
                y_scale.push_back(y / f);

                auto rgb_ints = rgb_img_.at<cv::Vec3b>(j, i);  // this is the RGB image from the sensor (e.g. Kinect/realsense) , but u need to align to depth! 
                
                float depth = 1.0 + scale * depth_raw[k];
                float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
                pcl::PointXYZRGB p;
                p.x = xyz[0]; 
                p.y = xyz[1]; 
                p.z = xyz[2]; 
                p.r = (int)rgb_ints[0];
                p.g = (int)rgb_ints[1];
                p.b = (int)rgb_ints[2];
                cloud->at(i,j) = p;
            }
        }
        Eigen::Affine3f T_off;        //deptj_link
        T_off.translation() << 0.0, 0.0, 0.0;//-0.225271001458,-0.115889430046,1.51970565319;
        T_off.linear() = Eigen::Quaternionf(0.500120937824,0.499887049198,0.499872893095,0.500119030476).toRotationMatrix();

        // Eigen::Affine3f T_BC;
        // T_BC.translation() << -1.42671394348,-0.160102918744,1.64789104462;//-1.42671489716 , -0.16010376811, 1.64789104462;
        // T_BC.linear() = Eigen::Quaternionf(0.411594420671,-0.596367001534,0.596368789673,-0.345370352268).toRotationMatrix();
        // //std::cerr<<T_BC.linear()<<std::endl;

        pcl::transformPointCloud (*cloud, *cloud, T_off);

        
        pcl::PCLPointCloud2 cloud_Generated;
        pcl::toPCLPointCloud2(*cloud, cloud_Generated);
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_Generated, output);
        
        vrep_sim_pointcloud_pub.publish(output); 
    }
} 