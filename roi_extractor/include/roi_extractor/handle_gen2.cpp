#include "../include/roi_extractor/handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    obj_cloud_pub_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/object", 1);
    handle_cloud_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/door_handle", 1);
    //grasp_pub_ = _nh.advertise<visualization_msgs::Marker>("/sampler/object", 1);

    yolo_detection_sub_ = _nh.subscribe("/darknet_ros_3d/bounding_boxes",1, &handle_sampler::reigon_cb, this);
    kinect_cloud_sub_ = _nh.subscribe("/camera/depth_registered/points",1, &handle_sampler::cloud_cb, this);
    // allocate variable siz
    roi_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualization_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    _nh.getParam("target_obj_num", target_object_num_);
    T_BC_.resize(4,4);

    // init params (Defalut)
    grasp_visualization_=false;
    detected_obj_num_ =0;

    //markerShape_ = visualization_msgs::Marker::ARROW;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_sampler::roi_filter_cloud(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*roi_cloud_, *local_cloud);

    Eigen::Affine3f T_LK;
    T_LK.translation() << -0.00014467 , 0.014879, 0.000155564;
    T_LK.linear() = Eigen::Quaternionf(-0.49997,0.49875,-0.49919,0.50208).toRotationMatrix();
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(local_cloud);
    boxFilter.setTransform(T_LK);
    boxFilter.filter(*local_cloud);
    local_cloud->header = roi_cloud_->header;
    return local_cloud;
}

void handle_sampler::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);
    cloud_msgs_.header = msg->header;
    roi_cloud_->header = cloud_msg->header;
    //std::cerr<<"frame id : "<<cloud_msgs_.header.frame_id<<std::endl;
    roi_cloud_ = cloud_msg;
}

void handle_sampler::reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    detected_obj_num_ = _objpose->bounding_boxes.size();
    if(detected_obj_num_ <= 0)
        return;
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
        copyPointCloud(*cloud, *result_.at(i).obj_cloud_);
    }

    obj_visualization();
    std::vector<geometry_msgs::Pose> graspPoints = getSolution(target_object_num_);

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

            point.r =  result_.at(i).obj_cloud_->at(j).r;
            point.g =  result_.at(i).obj_cloud_->at(j).g;
            point.b =  result_.at(i).obj_cloud_->at(j).b;

            // switch(i)
            // {
            // case 0:
            //     point.r = 0;
            //     point.g = 0;
            //     point.b = 255;
            //     break;
            // case 1:
            //     point.r = 0;
            //     point.g = 255;
            //     point.b = 0;
            //     break;
            // case 2:
            //     point.r = 255;
            //     point.g = 0;
            //     point.b = 0;
            //     break;
            // case 3:
            //     point.r = 255;
            //     point.g = 255;
            //     point.b = 0;
            //     break;
            // case 4:
            //     point.r = 0;
            //     point.g = 255;
            //     point.b = 255;
            //     break;
            // default:
            //     point.r = 0;
            //     point.g = 0;
            //     point.b = 255;
            //     break;
            // }
            visual_obj_cloud->push_back(point);
        }
    }
    pcl::PCLPointCloud2 cloud_Generated;
    visual_obj_cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*visual_obj_cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
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

std::vector<geometry_msgs::Pose> handle_sampler::getSolution(unsigned int _objNum)
{
    std::vector<geometry_msgs::Pose> handle_candidate;
    sensor_msgs::PointCloud2 cloud_visualization_;
    handle_candidate.resize(NUM_FOR_SAMPLING); 

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //cloud = result_.at(_objNum).obj_cloud_;
    copyPointCloud(*result_.at(_objNum).obj_cloud_,*cloud);

    pcl::PCLPointCloud2 cloud_Generated;
    cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
    handle_cloud_.publish(cloud_visualization_);



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
        std::cout<<"stick"<<std::endl;
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
        std::cout<<"handle"<<std::endl;
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

    int j= 0;

    // x min y min z min 1 -- >Homogenous Transform
    double min_dist = 5e3;
    Eigen::Vector2f base_position_xy(0,0);
    std::vector<Eigen::Vector2f> obj_min_pos;
    obj_min_pos.resize(clusters.size());

    ///////////////////// remove Outliers in Point Cloud ///////////////////////////////////

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        double dist;
        int num_point = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            
            pcl::PointXYZRGB point;
            point.x = cloud->points[*pit].x;
            point.y = cloud->points[*pit].y;
            point.z = cloud->points[*pit].z;
            point.r = 255;
            point.g = 0;
            point.b = 0;
            obj_min_pos.at(j)(0) += point.x;
            obj_min_pos.at(j)(1) += point.y; 
            num_point++;
            candidate_cloud->push_back(point);
        }

        obj_min_pos.at(j)(0) = obj_min_pos.at(j)(0) / num_point;
        obj_min_pos.at(j)(1) = obj_min_pos.at(j)(1) / num_point;

        dist = ( base_position_xy - obj_min_pos.at(j) ).norm();

        // at least point number has 30
        if( min_dist > dist && num_point > 30)
        {
            min_dist = dist;
            colored_cloud = candidate_cloud;  ////////////
        }
        j++;
    }

    /////////////////////////////////////////////////
    // plane segmentation: http://pointclouds.org/documentation/tutorials/planar_segmentation
    #include <pcl/sample_consensus/method_types.h>
    #include <pcl/sample_consensus/model_types.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/point_types.h>
    #include <pcl/features/normal_3d.h>

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (colored_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return (-1);
    }
    
    // Orientation: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
    pcl::PointCloud <pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx: inliers->indices){
        pcl::PointXYZ point;
        point.x = colored_cloud->points[idx].x;
        point.y = colored_cloud->points[idx].y;
        point.z = colored_cloud->points[idx].z;
        plane_cloud->push_back(point);
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (plane_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
        
    ///////////////////////////////////////////////////////////////////////////

    // pcl::PCLPointCloud2 cloud_Generated;
    // colored_cloud->header.frame_id = roi_cloud_->header.frame_id;
    // pcl::toPCLPointCloud2(*colored_cloud, cloud_Generated);
    // pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    // cloud_visualization_.header = cloud_msgs_.header;
    // handle_cloud_.publish(cloud_visualization_);

    ///////////////////////////////////////////////////////////////////////////

    pcl::RandomSample<pcl::PointXYZRGB> sample_points;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sample_points.setInputCloud(colored_cloud);
    sample_points.setSample(NUM_FOR_SAMPLING);
    sample_points.filter(*sampled_cloud);

    return handle_candidate;
}
