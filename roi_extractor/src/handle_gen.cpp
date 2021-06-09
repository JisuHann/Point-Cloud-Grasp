#include "../include/roi_extractor/handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    roi_cloud_pub_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/object", 1);
    handle_cloud_ = _nh.advertise<sensor_msgs::PointCloud2>("/sampler/door_handle", 1);
    grasp_pub_ = _nh.advertise<visualization_msgs::MarkerArray>("/grasp/candidate", 1);

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

    /////////////// down Sampling ////////////////////////////////////////////////////////////////////////

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(local_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*local_cloud);

    /////////////// StatisticalOutlierRemoval /////////////////////////////////////////////////////////////

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_f;
    sor_f.setInputCloud(local_cloud);
    sor_f.setMeanK(50);
    sor_f.setStddevMulThresh (1.0);
    sor_f.filter(*local_cloud);

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

    //////////////// for test form file///////////////////////////////////////////////////
    // std::vector<geometry_msgs::Pose> graspPoints = getSolution(target_object_num_);
    // //std::cout<<"e check"<<std::endl;
    // if(graspPoints.size() <= 0)
    //     return;

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

    ///////////////////////////////////////////////////////////////////////////////////////
}

void handle_sampler::reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    detected_obj_num_ = _objpose->bounding_boxes.size();
    if( detected_obj_num_ <= 0)
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

    if(graspPoints.size() <= 0)
        return;

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
        grasp_.scale.y = 0.05;
        grasp_.scale.z = 0.05;
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
    sensor_msgs::PointCloud2 cloud_visualization_;
    pcl::PCLPointCloud2 cloud_Generated;
    handle_candidate.resize(NUM_FOR_SAMPLING); 

    //////////////////////////////// Load form Kinect /////////////////////////////////////
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr CAD_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr CAD_process_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::VoxelGrid<pcl::PointXYZRGB> sor;


    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;


    std::vector<pcl::PointIndices> clusters;

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;


    // Plane Segmentation Obj////////////////////////////////////////////////
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr outliercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB,pcl::Normal> seg; 

    /////////////////////////////// Classify Door type ///////////////////////////////////////////
    // When you initialzie in switch-case you should use { }
    switch(result_.at(_objNum).door_type_) //result_.at(_objNum).door_type_
    {

    case STICK:
        std::cout<<"stick"<<std::endl;

        //////////////////////////////  Load door Handle Model /////////////////////////////////////////

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/sampleCloud/doorsample1.pcd", *CAD_cloud) == -1) //* load the file
        {
            ROS_INFO("Couldn't read file door Model \n");
            return handle_candidate;
        } 

        sor.setInputCloud(CAD_cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*CAD_cloud);

        RP <<  1,              0, 0,
                 0,  cos(90) , sin(90),
                 0,  -sin(90), cos(90);

        copyPointCloud(*CAD_cloud, *CAD_process_cloud);
        /////////////////// clustering with Normal Vector ////////////////////////////////////////////////

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

        break;
    case HANDLE:
        std::cout<<"handle"<<std::endl;
        ec.setClusterTolerance(0.01); // 2cm
        ec.setMinClusterSize(50); //100
        ec.setMaxClusterSize(99000000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusters);
        RP <<  cos(90), -sin(90), 0,
               sin(90),  cos(90), 0,
               0            ,  0            , 1;
        break;
    default:
        std::cout<<"No door Model Exist!!!"<<std::endl;
        break;
    }

    cloud->header.frame_id = roi_cloud_->header.frame_id;
    pcl::toPCLPointCloud2(*cloud, cloud_Generated);
    pcl_conversions::fromPCL(cloud_Generated, cloud_visualization_);
    cloud_visualization_.header = cloud_msgs_.header;
    handle_cloud_.publish(cloud_visualization_);

    //cloud = result_.at(_objNum).obj_cloud_;
    // copyPointCloud(*result_.at(_objNum).obj_cloud_,*cloud);

    //////////////////////////////////Load from Files (test)////////////////////////////////////////
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/min/catkin_ws/src/sampleCloud/doorsample1.pcd", *cloud) == -1) /home/min/catkin_ws/src/roi_extractor/CADMODEL/Refigerator_handle.pcd


    //save Point CLoud ////////////////////////////////////////////////////////////

    // if(cloud->size() > 0)
    // {
    //     std::string save_path = "/home/min/catkin_ws/src/result/grasp";
    //     std::string fileNum = std::to_string(saveNum);
    //     pcl::io::savePCDFileASCII (save_path+fileNum+".pcd", *cloud);
    //     saveNum++;
    // }

    ///////////////////////////////////////////////////////////////////////////////

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(CAD_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices (new std::vector <int>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(CAD_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*indices);


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
    //         point.x = cloud->points[*pit].x;
    //         point.y = cloud->points[*pit].y;
    //         point.z = cloud->points[*pit].z;


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

    // Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    // float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));


    // Eigen::Quaternionf quat(Eigen::AngleAxisf(theta, rotation_vector));
        
    ////////////////////////////////////////////////////////////////////////////

    /////////////////////////  Method to get Orientation with MomentOfIntertiaEstimation //////////////

    
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;

    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;

    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    Eigen::Quaternionf quat(rotational_matrix_OBB);

    //quat = rotation*quat.toRotationMatrix();
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    quat = RP*quat.toRotationMatrix();

    std::cout<<"Orientation "<<quat.toRotationMatrix()<<std::endl;
    pcl::RandomSample<pcl::PointXYZRGB> sample_points;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sample_points.setInputCloud(CAD_cloud);
    sample_points.setSample(NUM_FOR_SAMPLING);
    sample_points.filter(*sampled_cloud);

    /// Handle Parmeter를 만족하는 가?

    /// Give Handle Configuration
    for(unsigned int i = 0; i < sampled_cloud->size() ; i++)
    {
        handle_candidate.at(i).position.x = sampled_cloud->at(i).x;
        handle_candidate.at(i).position.y = sampled_cloud->at(i).y;
        handle_candidate.at(i).position.z = sampled_cloud->at(i).z;
        handle_candidate.at(i).orientation.w = quat.w();
        handle_candidate.at(i).orientation.x = quat.x();
        handle_candidate.at(i).orientation.y = quat.y();
        handle_candidate.at(i).orientation.z = quat.z();
    }


    //////////////////////////////////////////// Feature Matching with Current CLoud (SHOT)///////////////////////////////
    // SHOT is RUBUST POSE Estimator

    double resolution = computeCloudResolution(cloud);
    float model_ss_  = 0.01f*resolution;
    float scene_ss_ = 0.03f*resolution;
    float rf_rad_ = 0.015f*resolution;
    float descr_rad_ = 0.02f*resolution;
    float cg_size_ = 0.01f*resolution;
    float cg_thresh_ = 5.0f*resolution;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
    
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352>);
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors (new pcl::PointCloud<pcl::SHOT352>);

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setKSearch(10);
    norm_est.setInputCloud(CAD_process_cloud);
    norm_est.compute(*model_normals);

    norm_est.setInputCloud(cloud);
    norm_est.compute(*scene_normals);

    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud(CAD_process_cloud);
    uniform_sampling.setRadiusSearch(model_ss_);
    uniform_sampling.filter(*model_keypoints);

    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(scene_ss_);
    uniform_sampling.filter(*scene_keypoints);

    pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch(descr_rad_);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(CAD_process_cloud);
    descr_est.compute(*model_descriptors);

    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(cloud);
    descr_est.compute (*scene_descriptors);

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.

    for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame>);
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame>);

    pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (CAD_process_cloud);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB,pcl::ReferenceFrame,pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);


    if(rototranslations.size() <= 0)
    {
       std::cout<<"Matching Error No Fine Model Finded"<<std::endl;
       return std::vector<geometry_msgs::Pose>(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[0].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[0].block<3,1>(0, 3);

    for(unsigned int i = 0; i < sampled_cloud->size() ; i++)
    {
        Eigen::Affine3f T_OO;
        T_OO.translation() << handle_candidate.at(i).position.x,handle_candidate.at(i).position.y,handle_candidate.at(i).position.z;
        T_OO.linear() = Eigen::Quaternionf(handle_candidate.at(i).orientation.w
            ,handle_candidate.at(i).orientation.x,handle_candidate.at(i).orientation.y,handle_candidate.at(i).orientation.z).toRotationMatrix();

        Eigen::Affine3f T_icp;
        T_icp.translation() = translation;
        T_icp.linear() = rotation;

        T_OO = T_OO * T_icp;

        Eigen::Translation3f trans = Eigen::Translation3f(T_OO.translation());
        Eigen::Quaternionf quat = Eigen::Quaternionf(T_OO.rotation());


        handle_candidate.at(i).position.x = trans.x();
        handle_candidate.at(i).position.x = trans.x();
        handle_candidate.at(i).position.x = trans.x();

        handle_candidate.at(i).orientation.x = quat.x();
        handle_candidate.at(i).orientation.y = quat.y();
        handle_candidate.at(i).orientation.z = quat.z();
        handle_candidate.at(i).orientation.w = quat.w();
    }


    /////////////////////////////// Output Recifyed /////////////////////////////////////////////////////
    return handle_candidate;
}

double handle_sampler::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}