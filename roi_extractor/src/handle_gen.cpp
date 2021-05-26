#include "../include/roi_extractor/handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    yolo_detection_sub_ = _nh.subscribe("/darknet_ros_3d/bounding_boxes",10, &handle_sampler::reigon_cb, this);

    // allocate variable siz
    roi_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    T_BC_.resize(4,4);

    // init params (Defalut)
    grasp_visualization_=false;
    detected_obj_num_ =0;
}

void handle_sampler::reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    detected_obj_num_ = _objpose->bounding_boxes.size();
    result_.resize(detected_obj_num_);
    int tmp_obj_num = detected_obj_num_;
    
    for(unsigned int i = 0 ; i < detected_obj_num_; i++)
    {
        if(_objpose->bounding_boxes.at(i).Class == "door1")
            result_.at(i).door_type_ = STICK;
        else if(_objpose->bounding_boxes.at(i).Class == "door2")
            result_.at(i).door_type_ = HANDLE;
        else
        {
            std::cout <<"Door Handle type Not Founded!"<<std::endl;
            tmp_obj_num--;
            continue;
        }
        result_.at(i).obj_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        result_.at(i).result_candidate.resize(10);

        //// ROI Cluod Processing 

        /////////////////////////
        result_.at(i).obj_cloud_ = cloud;
    }
}

Eigen::Matrix4f handle_sampler::Frame2Eigen(KDL::Frame &frame)
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

void handle_sampler::roi_filter_cloud()
{

}

void handle_sampler::InitRobotKinematics(KDL::JntArray _robot_joint_val, KDL::Chain _robot_chain)
{
    FK_chain_ = _robot_chain;
    robot_joint_val_ = KDL::JntArray(FK_chain_.getNrOfJoints());
    for(unsigned int i = 0; i <FK_chain_.getNrOfJoints(); i++ )
        robot_joint_val_(i) = _robot_joint_val(i);

    KDL::ChainFkSolverPos_recursive Fksolver = KDL::ChainFkSolverPos_recursive(FK_chain_);

    if(Fksolver.JntToCart(robot_joint_val_,T_BC_Frame) < 0)
        std::cerr<<"Fk about robot Model Failed!!!!"<<std::endl;

    // T_BO = T_BC * T_CO ---> External Params
    T_BC_ = Frame2Eigen(T_BC_Frame);
}

void handle_sampler::grasp_candidate_gen()
{   
    // filering point cloud to make roi includes
    roi_filter_cloud();

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(roi_cloud_);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud(cloud_msg);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 2.0);
    // pass.filter(*indices);

    // std::vector<pcl::PointIndices> clusters;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	// ec.setClusterTolerance(0.01); // 2cm
	// ec.setMinClusterSize(50); //100
	// ec.setMaxClusterSize(99000000);
	// ec.setSearchMethod(tree);
	// ec.setInputCloud(cloud_msg);
	// ec.extract(clusters);

    // pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    // reg.setMinClusterSize(50);
    // reg.setMaxClusterSize(1000000);
    // reg.setSearchMethod(tree);
    // reg.setNumberOfNeighbours(30);
    // reg.setInputCloud(cloud_msg);
    // //reg.setIndices (indices);
    // reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(0.7 / 180.0 * M_PI);
    // reg.setCurvatureThreshold(0.1);
}
