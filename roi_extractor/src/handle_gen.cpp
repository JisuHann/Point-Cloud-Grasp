#include "../include/roi_extractor/handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    obj_cloud_pub_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/object", 1);

    yolo_detection_sub_ = _nh.subscribe("/darknet_ros_3d/bounding_boxes",1, &handle_sampler::reigon_cb, this);
    kinect_cloud_sub_ = _nh.subscribe("/camera/depth/color/points",1, &handle_sampler::cloud_cb, this);
    // allocate variable siz
    roi_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualization_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    T_BC_.resize(4,4);

    // init params (Defalut)
    grasp_visualization_=false;
    detected_obj_num_ =0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_sampler::roi_filter_cloud(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*roi_cloud_, *local_cloud);
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(local_cloud);
    boxFilter.filter(*local_cloud);
    return local_cloud;
}

void handle_sampler::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);

    roi_cloud_ = cloud_msg;
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
        {
            result_.at(i).door_type_ = STICK;
        } 
        else if(_objpose->bounding_boxes.at(i).Class == "door2")
        {
            result_.at(i).door_type_ = HANDLE;
        }
        else
        {
            std::cout <<"Door Handle type Not Founded!"<<std::endl;
            tmp_obj_num--;
            continue;
        }
        result_.at(i).obj_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        result_.at(i).result_candidate.resize(NUM_FOR_SAMPLING);

        //// ROI Cluod Processing /////////////////////////////////////////////////
        Eigen::Vector4f minPoint;
        minPoint[0] = _objpose->bounding_boxes.at(i).xmin;
        minPoint[1] = _objpose->bounding_boxes.at(i).ymin;
        minPoint[2] = _objpose->bounding_boxes.at(i).zmin;
        minPoint[3] = 1.0;
        Eigen::Vector4f maxPoint;
        maxPoint[0] = _objpose->bounding_boxes.at(i).xmax;
        maxPoint[1] = _objpose->bounding_boxes.at(i).ymax;
        maxPoint[2] = _objpose->bounding_boxes.at(i).zmax;
        maxPoint[3] = 1.0;

        cloud = roi_filter_cloud(minPoint,maxPoint);
        ///////////////////////////////////////////////////////////////////////////
        result_.at(i).obj_cloud_ = cloud;
    }

    obj_visualization();
}

void handle_sampler::obj_visualization() 
{
    sensor_msgs::PointCloud2 cloud_visualization_;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr visual_obj_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(unsigned int i = 0; i < detected_obj_num_;i++)
    {
        for(unsigned int j =0 ; j < result_.at(i).obj_cloud_->size() ; j++ )
        {
            pcl::PointXYZRGB point;
            point.x =  result_.at(i).obj_cloud_->at(j).x;
            point.y =  result_.at(i).obj_cloud_->at(j).y;
            point.z =  result_.at(i).obj_cloud_->at(j).z;
            switch(i)
            {
            case 0:
                point.r = 0;
                point.g = 0;
                point.b = 255;
                break;
            case 1:
                point.r = 0;
                point.g = 255;
                point.b = 0;
                break;
            case 2:
                point.r = 255;
                point.g = 0;
                point.b = 0;
                break;
            case 3:
                point.r = 255;
                point.g = 255;
                point.b = 0;
                break;
            case 4:
                point.r = 0;
                point.g = 255;
                point.b = 255;
                break;
            default:
                point.r = 0;
                point.g = 0;
                point.b = 255;
                break;
            }
            visual_obj_cloud->push_back(point);
        }
    }
    pcl::PCLPointCloud2 cloud_Generated;
    pcl::toPCLPointCloud2(*visual_obj_cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);

    obj_cloud_pub_.publish(cloud_visualization_);
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

void handle_sampler::InitRobotKinematics(KDL::JntArray _nominal, KDL::Chain _robot_chain)
{
    FK_chain_ = _robot_chain;
    robot_joint_val_ = KDL::JntArray(FK_chain_.getNrOfJoints());
    for(unsigned int i = 0; i <FK_chain_.getNrOfJoints(); i++ )
        robot_joint_val_(i) = _nominal(i);

    KDL::ChainFkSolverPos_recursive Fksolver = KDL::ChainFkSolverPos_recursive(FK_chain_);

    if(Fksolver.JntToCart(robot_joint_val_,T_BC_Frame) < 0)
        std::cerr<<"Fk about robot Model Failed!!!!"<<std::endl;

    // T_BO = T_BC * T_CO ---> External Params
    T_BC_ = Frame2Eigen(T_BC_Frame);
}

geometry_msgs::Pose handle_sampler::getSolution(unsigned int _objNum)
{   
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud = result_.at(_objNum).obj_cloud_;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*indices);

    std::vector<pcl::PointIndices> clusters;

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

    // When you initialzie in switch-case you should use { }
    switch(result_.at(_objNum).door_type_)
    {

    case STICK:
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud);
        //reg.setIndices (indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(0.7 / 180.0 * M_PI);
        reg.setCurvatureThreshold(0.1);
        reg.extract(clusters);
        break;
    case HANDLE:
        ec.setClusterTolerance(0.01); // 2cm
        ec.setMinClusterSize(50); //100
        ec.setMaxClusterSize(99000000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusters);
        break;
    default:
        std::cout<<"No door Model Exist!!!"<<std::endl;
        break;
    }


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_copy(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j= 0;
    int max_X = 0;
    int idxCloud = 0;
    double min_z = 0.0;
    int numiter = 0;

    ///////////////////// remove Outliers in Point Cloud ///////////////////////////////////

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[*pit].x;
            point.y = cloud->points[*pit].y;
            point.z = cloud->points[*pit].z;
            point.r = 255;
            point.g = 0;
            point.b = 0;

            if (j == 1) //Lime	#00FF00	(0,255,0)
            {
               point.r = 0;
               point.g = 255;
               point.b = 0;
            }
            if(j==1)
            {
               colored_cloud->push_back(point);
               min_z += point.z;
               numiter++;
            }
        }
        j++;
    }

    ///////////////////////////////////////////////////////////////////////////

   if(numiter == 0)
      min_z = 0.0;
   else
      min_z = min_z/numiter;
}
