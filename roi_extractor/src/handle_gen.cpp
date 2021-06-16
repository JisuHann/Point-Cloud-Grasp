#include "../include/roi_extractor/handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    roi_cloud_pub_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/object", 1);
    handle_cloud_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/door_handle", 1);
    grasp_pub_ = _nh.advertise<visualization_msgs::MarkerArray>("/grasp/candidate", 1);
    grasp_test = _nh.advertise<sensor_msgs::PointCloud2>("/grasp/test", 1);

    yolo_detection_sub_ = _nh.subscribe("/darknet_ros_3d/bounding_boxes",1, &handle_sampler::reigon_cb, this);
    kinect_cloud_sub_ = _nh.subscribe("/k4a/depth_registered/points",1, &handle_sampler::cloud_cb, this);

    // allocate variable siz
    roi_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualization_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // init params (Defalut)
    original_color_visualization_=false;
    detected_obj_num_ =0;
    saveNum = 0;
    _nh.getParam("target_obj_num", target_object_num_);
    _nh.getParam("original_color_visualization",original_color_visualization_);

    ///////////////////// allocate  Matrix //////////////////////////////////////////////////////////////

    T_BC_.resize(4,4);

    //////////////////////////////////////////////////////////////////////////////////////////////////////
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_sampler::roi_filter_cloud(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*roi_cloud_, *local_cloud);

    Eigen::Affine3f T_LK;

    //////////////////   Realsense depth to optical link ///////////////////////////////////////////////////

    // Transform Matrix to depth link (realsense D435)
    // T_LK.translation() << -0.00014467 , 0.014879, 0.000155564;
    // T_LK.linear() = Eigen::Quaternionf(-0.49997,0.49875,-0.49919,0.50208).toRotationMatrix();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////// Transform Matrix to depth Link Azure Kinect (Base to rgb_camera_link) /////////////////////

    T_LK.translation() << -0.0039076 , -0.0320149, -0.000327477;
    T_LK.linear() = Eigen::Quaternionf(0.498297,-0.49927,0.501692,-0.500734).toRotationMatrix();

    //////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////// ROI (Crop BOX Filter) //////////////////////////////////////////

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(local_cloud);
    boxFilter.setTransform(T_LK);
    boxFilter.filter(*local_cloud);
    local_cloud->header = roi_cloud_->header;
    return local_cloud;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
}

void handle_sampler::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);
    cloud_msgs_.header = msg->header;
    roi_cloud_->header = cloud_msg->header;

    grasp_.header.frame_id = msg->header.frame_id;

    //std::cerr<<"frame id : "<<cloud_msgs_.header.frame_id<<std::endl; --> rgb_camer_link (Azure Kinect)
    roi_cloud_ = cloud_msg;

    //////////////// for test form file/////////////////////////////////////////////////////////////////

    // std::vector<geometry_msgs::Pose> graspPoints = getSolution(target_object_num_);

    // if(graspPoints.size() <= 0)
    //     return;
    // ///////////////////////////////////////////////////////////////////////////////////////

    // for(unsigned int i = 0; i < graspPoints.size(); i++)
    // {
    //     grasp_.type = visualization_msgs::Marker::ARROW;
    //     grasp_.action = visualization_msgs::Marker::ADD;
    //     grasp_.id = i;
    //     grasp_.header.stamp = ros::Time::now();

    //     grasp_.pose.position.x = graspPoints.at(i).position.x;
    //     grasp_.pose.position.y = graspPoints.at(i).position.y;
    //     grasp_.pose.position.z = graspPoints.at(i).position.z;

    //     grasp_.pose.orientation.x = graspPoints.at(i).orientation.x;
    //     grasp_.pose.orientation.y = graspPoints.at(i).orientation.y;
    //     grasp_.pose.orientation.z = graspPoints.at(i).orientation.z;
    //     grasp_.pose.orientation.w = graspPoints.at(i).orientation.w;

    //     grasp_.scale.x = 0.05;
    //     grasp_.scale.y = 0.005;
    //     grasp_.scale.z = 0.005;
    //     grasp_.color.a = 1.0; // Don't forget to set the alpha!
    //     grasp_.color.r = 0.0;
    //     grasp_.color.g = 1.0;
    //     grasp_.color.b = 0.0;
    //     grasps_.markers.push_back(grasp_);
    // }

    // grasp_pub_ .publish(grasps_);
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////
}

void handle_sampler::reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    detected_obj_num_ = _objpose->bounding_boxes.size();
    if( detected_obj_num_ <= 0)
    {
        std::cout<<"NO Object Detected"<<std::endl;
        return;
    }

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

    // if(cloud->size() > 0)
    // {
    //     std::string save_path = "/home/min/catkin_ws/src/result/grasp";
    //     std::string fileNum = std::to_string(saveNum);
    //     pcl::io::savePCDFileASCII (save_path+fileNum+".pcd", *cloud);
    //     saveNum++;
    // }


    std::vector<geometry_msgs::Pose> graspPoints = getSolution(target_object_num_);
    std::cout<<"grasp here"<<graspPoints.size()<<std::endl;
    if(graspPoints.size() <= 0)
    {
        std::cout<<"No Grasp Point Detected!!!!!!!!!!!"<<std::endl;
        return;
    }

    for(unsigned int i = 0; i < graspPoints.size(); i++)
    {
        grasp_.type = visualization_msgs::Marker::ARROW;
        grasp_.action = visualization_msgs::Marker::ADD;
        grasp_.id = i;
        grasp_.header.stamp = ros::Time::now();

        grasp_.pose.position.x = graspPoints.at(i).position.x;
        grasp_.pose.position.y = graspPoints.at(i).position.y;
        grasp_.pose.position.z = graspPoints.at(i).position.z;

        grasp_.pose.orientation.x = graspPoints.at(i).orientation.x;
        grasp_.pose.orientation.y = graspPoints.at(i).orientation.y;
        grasp_.pose.orientation.z = graspPoints.at(i).orientation.z;
        grasp_.pose.orientation.w = graspPoints.at(i).orientation.w;

        grasp_.scale.x = 0.05;
        grasp_.scale.y = 0.005;
        grasp_.scale.z = 0.005;
        grasp_.color.a = 1.0; // Don't forget to set the alpha!
        grasp_.color.r = 0.0;
        grasp_.color.g = 1.0;
        grasp_.color.b = 0.0;
        grasps_.markers.push_back(grasp_);
    }

    grasp_pub_ .publish(grasps_);
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

            if(!original_color_visualization_)
            {
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
            }
            else
            {
                point.r =  result_.at(i).obj_cloud_->at(j).r;
                point.g =  result_.at(i).obj_cloud_->at(j).g;
                point.b =  result_.at(i).obj_cloud_->at(j).b;
            }
            visual_obj_cloud->push_back(point);
        }
    }

    pcl::PCLPointCloud2 cloud_Generated;
    visual_obj_cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*visual_obj_cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
    roi_cloud_pub_.publish(cloud_visualization_);
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
    Eigen::Quaternionf orientation;

    sensor_msgs::PointCloud2 cloud_visualization_;
    pcl::PCLPointCloud2 cloud_Generated;
    handle_candidate.resize(NUM_FOR_SAMPLING); 

    //////////////////////////////// Load form Kinect /////////////////////////////////////

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr CAD_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr CAD_cloud_ICP(new pcl::PointCloud<pcl::PointXYZRGB>);

    //////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////Load from Files (test)////////////////////////////////////////

    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/sampleCloud/doorsample1.pcd", *cloud) == -1) /home/min/catkin_ws/src/roi_extractor/CADMODEL/Refigerator_handle.pcd
    
    // if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/sampleCloud/doorsample1.pcd", *CAD_cloud) == -1) //* load the file
    // {
    //     ROS_INFO("Couldn't read file door Model \n");
    //     return handle_candidate;
    // }

    if(result_.at(_objNum).door_type_ == STICK) //result_.at(_objNum).door_type_
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/MODEL/Handle.pcd", *CAD_cloud) == -1) //* load the file
        {
            ROS_INFO("Couldn't read file CAD FILES \n");
            return handle_candidate;
        }
    }
    else if(result_.at(_objNum).door_type_ == HANDLE)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/MODEL/refri.pcd", *CAD_cloud) == -1) //* load the file
        {
            ROS_INFO("Couldn't read file CAD FILES \n");
            return handle_candidate;
        }
    }


    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(CAD_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*CAD_cloud);

    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    cloud = result_.at(_objNum).obj_cloud_;
    copyPointCloud(*result_.at(_objNum).obj_cloud_,*cloud);
    CAD_cloud->header = roi_cloud_->header;
    CAD_cloud_ICP->header = CAD_cloud->header;
    copyPointCloud(*CAD_cloud,*CAD_cloud_ICP);

    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(CAD_cloud);
    // sor.setLeafSize(0.01f, 0.01f, 0.01f);
    // sor.filter(*CAD_cloud);
    // /*

    /////////////////////save Point CLoud ////////////////////////////////////////

    // if(cloud->size() > 0)
    // {
    //     std::string save_path = "/home/min/catkin_ws/src/result/grasp";
    //     std::string fileNum = std::to_string(saveNum);
    //     pcl::io::savePCDFileASCII (save_path+fileNum+".pcd", *cloud);
    //     saveNum++;
    // }

    ///////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////


    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(CAD_cloud);
    // sor.setLeafSize(0.01f, 0.01f, 0.01f);
    // sor.filter(*CAD_cloud);

    //std::cout<<"file Cloud size :  "<<cloud->size()<<std::endl;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(CAD_cloud);
    normal_estimator.setKSearch(30);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices (new std::vector <int>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(CAD_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*indices);

    std::vector<pcl::PointIndices> clusters;

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;


    // Plane Segmentation Obj////////////////////////////////////////////////
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr outliercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB,pcl::Normal> seg; 

    CAD_cloud_ICP->header.frame_id = roi_cloud_->header.frame_id;
    CAD_cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*CAD_cloud_ICP, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
    grasp_test.publish(cloud_visualization_);

    cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
    handle_cloud_.publish(cloud_visualization_);


    // When you initialzie in switch-case you should use { }
    switch(result_.at(_objNum).door_type_) //result_.at(_objNum).door_type_
    {

    case STICK:
        std::cout<<"stick"<<std::endl;

        seg.setOptimizeCoefficients (true);  // Optional
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //PLANE 모델 사용
        seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용 
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01); //determines how close a point must be to the model in order to be considered an inlier
        seg.setInputCloud(CAD_cloud);
        seg.setInputNormals(normals);
        seg.segment (*inliers, *coefficients);

        extract.setInputCloud(CAD_cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);
        extract.filter(*CAD_cloud);
        RP <<  1,              0, 0,
                 0,  cos(90) , sin(90),
                 0,  -sin(90), cos(90);
        orientation = Eigen::Quaternionf(0.503903,-0.495908,-0.509718,-0.49025);
        break;
    case HANDLE:
        std::cout<<"handle"<<std::endl;

        seg.setOptimizeCoefficients (true);  // Optional
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //PLANE 모델 사용
        seg.setMethodType(pcl::SAC_RANSAC);  //RANSAC 방법 사용 
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.1); //determines how close a point must be to the model in order to be considered an inlier
        seg.setInputCloud(CAD_cloud);
        seg.setInputNormals(normals);
        seg.segment (*inliers, *coefficients);

        extract.setInputCloud(CAD_cloud);
        extract.setIndices(inliers);
        extract.setNegative (true);
        extract.filter(*CAD_cloud);
        // ec.setClusterTolerance(0.08); // 2cm
        // ec.setMinClusterSize(10); //100
        // ec.setMaxClusterSize(99000000);
        // ec.setSearchMethod(tree);
        // ec.setInputCloud(CAD_cloud);
        // ec.extract(clusters);
        //orientation = Eigen::Quaternionf(0.702972,-0.0097654,-0.711085,0.00965399);
        orientation = Eigen::Quaternionf(0.700804,-0.0112836,0.713179,-0.0110878);
        break;
    default:
        std::cout<<"No door Model Exist!!!"<<std::endl;
        break;
    }

    ///////////////////////////////////////////////////////////////////////////////

    //pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // x min y min z min 1 -- >Homogenous Transform

    // double min_dist = 5e3;

    //Eigen::Vector2f base_position_xy(0,0);
    //std::vector<Eigen::Vector2f> obj_min_pos;

    // if(clusters.size() <= 0)
    // {
    //     std::cout<<"no Cloud Founded"<<std::endl;
    //     return handle_candidate;
    // }
    //obj_min_pos.resize(clusters.size());

    ///////////////////// remove Outliers in Point Cloud ///////////////////////////////////

    // int j= 0;

    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    // {
    //     //pcl::PointCloud <pcl::PointXYZRGB>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     double dist;
    //     int num_point = 0;
    //     for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    //     {
            
    //         pcl::PointXYZRGB point;
    //         point.x = CAD_cloud->points[*pit].x;
    //         point.y = CAD_cloud->points[*pit].y;
    //         point.z = CAD_cloud->points[*pit].z;


    //         ///////////////////////// Test Code ////////////////////////////////////////////////////////////////////////////////////////////
    //         switch(j)
    //         {
    //         case 0:
    //             point.r = 0;
    //             point.g = 0;
    //             point.b = 255;
    //             break;
    //         case 1:
    //             point.r = 0;
    //             point.g = 255;
    //             point.b = 0;
    //             break;
    //         case 2:
    //             point.r = 255;
    //             point.g = 0;
    //             point.b = 0;
    //             break;
    //         case 3:
    //             point.r = 255;
    //             point.g = 255;
    //             point.b = 0;
    //             break;
    //         case 4:
    //             point.r = 0;
    //             point.g = 255;
    //             point.b = 255;
    //             break;
    //         default:
    //             point.r = 0;
    //             point.g = 255;
    //             point.b = 0;
    //             break;
    //         }

    //         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //         // point.r = 255;
    //         // point.g = 0;
    //         // point.b = 0;
    //         // obj_min_pos.at(j)(0) += point.x;
    //         // obj_min_pos.at(j)(1) += point.y; 
    //         num_point++;
            
    //         colored_cloud->push_back(point);
    //     }

    //     // obj_min_pos.at(j)(0) = obj_min_pos.at(j)(0) / num_point;
    //     // obj_min_pos.at(j)(1) = obj_min_pos.at(j)(1) / num_point;

    //     // dist = ( base_position_xy - obj_min_pos.at(j) ).norm();

    //     // // at least point number has 30
    //     // if( min_dist > dist && num_point > 30)
    //     // {
    //     //     min_dist = dist;
    //     //     colored_cloud = candidate_cloud;  ////////////
    //     // }
    //     j++;
    // }


    ///////////////////////////////////////////////////////////////////////////
    //colored_cloud = candidate_cloud; 

    //copyPointCloud(*candidate_cloud, *colored_cloud);
    // 올바르게 Handle만 추출 되었는가???? 검증하는 Publisher

    

    //std::cout<<"seg ended  :"<<cloud->size()<<std::endl;

    /////////////////////////// plane segmentation to get orientation /////////////////////////////
    // Find the planar coefficients for floor plane


    // pcl::SACSegmentation<pcl::PointXYZRGB> plaen_seg;
    // plaen_seg.setOptimizeCoefficients (true);
    // plaen_seg.setModelType (pcl::SACMODEL_PLANE);
    // plaen_seg.setMethodType (pcl::SAC_RANSAC);
    // plaen_seg.setDistanceThreshold (0.01);
    // plaen_seg.setInputCloud (cloud);
    // plaen_seg.segment (*inliers, *coefficients);

    // Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector;

    // floor_plane_normal_vector[0] = coefficients->values[0];
    // floor_plane_normal_vector[1] = coefficients->values[1];
    // floor_plane_normal_vector[2] = coefficients->values[2];

    // xy_plane_normal_vector[0] = 0.0;
    // xy_plane_normal_vector[1] = 0.0;
    // xy_plane_normal_vector[2] = 1.0;

    // Eigen::Vector3f _vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    // float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));


    // Eigen::Quaternionf quat(Eigen::AngleAxisf(theta, rotation_vector));
        
    ////////////////////////////////////////////////////////////////////////////

    /////////////////////////  Method to get Orientation with MomentOfIntertiaEstimation //////////////

    
    // pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    // feature_extractor.setInputCloud (CAD_cloud);
    // feature_extractor.compute ();

    // std::vector <float> moment_of_inertia;
    // std::vector <float> eccentricity;

    // pcl::PointXYZRGB min_point_AABB;
    // pcl::PointXYZRGB max_point_AABB;
    // pcl::PointXYZRGB min_point_OBB;
    // pcl::PointXYZRGB max_point_OBB;
    // pcl::PointXYZRGB position_OBB;

    // Eigen::Matrix3f rotational_matrix_OBB;
    // float major_value, middle_value, minor_value;
    // Eigen::Vector3f major_vector, middle_vector, minor_vector;
    // Eigen::Vector3f mass_center;

    // feature_extractor.getMomentOfInertia(moment_of_inertia);
    // feature_extractor.getEccentricity(eccentricity);
    // feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    // feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    // feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    // feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    // feature_extractor.getMassCenter(mass_center);

    // Eigen::Quaternionf quat(rotational_matrix_OBB);
    

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // quat = RP*quat.toRotationMatrix();

    // std::cout<<"Orientation "<<quat.toRotationMatrix()<<std::endl;
    pcl::RandomSample<pcl::PointXYZRGB> sample_points;

    sample_points.setInputCloud(CAD_cloud);
    sample_points.setSample(NUM_FOR_SAMPLING);
    sample_points.filter(*CAD_cloud);

    //////////////////////////////////////////////////////  ICP to get transform Matrix /////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud);  
    icp.setInputTarget(CAD_cloud_ICP);
    icp.setMaximumIterations(10);
    icp.align(*cloud);

    if (icp.hasConverged ())
    {
        //std::cout << "\n Is score is " << icp.getFitnessScore () << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<float>();
    }
    else
    {
        std::cout<<"\n YOLO ROI is not Stable in this Stage\n"<<std::endl;;
        return handle_candidate;
    }

    Eigen::Matrix3f rotation = transformation_matrix.block<3,3>(0, 0);
    Eigen::Vector3f translation = transformation_matrix.block<3,1>(0, 3);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Give Handle Configuration   
    for(unsigned int i = 0; i < CAD_cloud->size() ; i++)
    {

        Eigen::Affine3f T_OO;
        T_OO.translation() << CAD_cloud->at(i).x,CAD_cloud->at(i).y,CAD_cloud->at(i).z;
        T_OO.linear() = orientation.toRotationMatrix();

        Eigen::Affine3f T_icp;
        T_icp.translation() = translation;
        T_icp.linear() = rotation;

        std::cout<<"transform\n"<<T_icp.matrix()<<std::endl;

        T_OO =  T_icp.inverse()*T_OO ;

        Eigen::Translation3f trans = Eigen::Translation3f(T_OO.translation());
        Eigen::Quaternionf quat = Eigen::Quaternionf(T_OO.rotation());


        handle_candidate.at(i).position.x = trans.x();
        handle_candidate.at(i).position.y = trans.y();
        handle_candidate.at(i).position.z = trans.z();

        handle_candidate.at(i).orientation.x = quat.x();
        handle_candidate.at(i).orientation.y = quat.y();
        handle_candidate.at(i).orientation.z = quat.z();
        handle_candidate.at(i).orientation.w = quat.w();
    }


    //////////////////  Corresspoding Gruping ///////////////////////////////////////////////

    return handle_candidate;
}
