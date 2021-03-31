#include <ros/ros.h>
#include <vector>
#include <math.h>

#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Pose.h>

//PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>

//tf includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

//includes Eigens
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

//#include "roi_extractor/boxfilter.hpp"


// services ,msgs includes
//#include <roi_extractor_msg/roiExtract.h> custom msgs
//#include <assembly_msgs/GetOccupancyVoxel.h>
#include "gb_visual_detection_3d_msgs/BoundingBoxes3d.h"


sensor_msgs::PointCloud2 g_cloud;
ros::Publisher g_pub_filtered;
ros::Publisher g_pub_test;
bool smapore = false;
/*
geometry_msgs::Pose g_Pose;
sensor_msgs::PointCloud2 g_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher g_pub_filtered;
ros::Publisher g_pub_clone;

void cloud_cb()
{
    //reset
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud_init (new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*g_cloud_filtered, *g_cloud_init);
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(g_cloud,*cloud_msg);

    //Data Saving Place
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr restored_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    Eigen::Affine3f T_BO, T_OB, T_CB, T_BC;
    T_BO.translation() << g_Pose.position.x, g_Pose.position.y, g_Pose.position.z;
    T_BO.linear() = Eigen::Quaternionf(g_Pose.orientation.w,g_Pose.orientation.x,g_Pose.orientation.y,g_Pose.orientation.z).toRotationMatrix();
    std::cout << "T_BO"<<std::endl;
    std::cout << T_BO.matrix()<<std::endl;
    T_OB = T_BO.inverse();
    std::cout << "T_OB"<<std::endl;
    std::cout << T_OB.matrix() <<std::endl;
    T_BC.matrix() << 0.0324386,  0.9170405,  0.3974727, 0.021856,
                  0.9990817, -0.0408897,  0.0128025, 0.0439238,
                  0.0279930,  0.3966924, -0.9175247, 2.55231,
                  0,          0,          0,          1;
    T_CB = T_BC.inverse();
    pcl::transformPointCloud (*cloud_msg, *transformed_cloud, T_OB * T_BC);
    // pass Through Filter cloud  --> rnage filter
    // ROI SET
    // 50 x 50 x 50 ROI
    Eigen::Vector4f minPoint;
    minPoint[0] = -0.4;
    minPoint[1] = -0.4;
    minPoint[2] = -0.3;
    minPoint[3] = 1;
    Eigen::Vector4f maxPoint;
    maxPoint[0] = 0.4;
    maxPoint[1] = 0.4;
    maxPoint[2] = 0.3;
    maxPoint[3] = 1;
    
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setInputCloud(transformed_cloud);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.filter(*transformed_cloud);

    //voxel Filter
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(transformed_cloud);
    sor.setLeafSize(0.04, 0.04, 0.04);
    sor.filter(*transformed_cloud);
    std::cout << "maxboxcor " << sor.getMaxBoxCoordinates() << std::endl;
    std::cout << "minboxcor " << sor.getMinBoxCoordinates() << std::endl;
    std::cout << "size: " << sor.getMaxBoxCoordinates() - sor.getMinBoxCoordinates() << std::endl;
    
    //StatisticalOutlierRemoval Filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> rmv;
    rmv.setInputCloud(transformed_cloud);
    rmv.setMeanK(50);
    rmv.setStddevMulThresh(1.0);
    rmv.filter(*transformed_cloud);

    int len_points = 

    Eigen::Tensor<int, 3> return_matrix(16, 16, 16);
    return_matrix.setZero();

    pcl::transformPointCloud (*transformed_cloud, *restored_cloud,  T_CB * T_BO);


    // //must be acess in base_frame (지금은 zvid_pc_50704 )
    // geometry_msgs::Pose transformedPose = g_Pose;

    // //Homogenous Transform Matrix
    // Eigen::Matrix4f BaseTozivid;
    // //zividToBase << 0.0324386,  0.9170405,  0.3974727, 0.021856,
    // //               0.9990817, -0.0408897,  0.0128025, 0.0439238,
    // //               0.0279930,  0.3966924, -0.9175247, 2.55231,
    // //               0,          0,          0,          1;
    // //inverse Matrix Base to zvid 들어온 좌표를 zivid로 변환
    // BaseTozivid << 0.032439,	 0.999082,	 0.027993,	-0.116039,
    //                0.917040,	-0.040890,	 0.396692,	-1.030729,
    //                0.397473,	 0.012803,	-0.917525,	 2.332558,
    //                0,	         0,  	     0,	         1;
                   
    // // You can either apply transform_1 or transform_2; they are the same

    // //Min and Max refer to the two diagonal points of the cube. Each point is represented by a four-dimensional vector, usually the last one is 1. (I don’t know why there are four, the big god knows to give an answer)
    // Eigen::Vector4f boxTranslatation;
    // boxTranslatation[0]=g_Pose.position.x;
    // boxTranslatation[1]=g_Pose.position.y;
    // boxTranslatation[2]=g_Pose.position.z;
    // boxTranslatation[3]=1;

    // boxTranslatation = BaseTozivid*boxTranslatation;

    // Eigen::Vector3f posTranslate;
    // posTranslate[0] = boxTranslatation[0];
    // posTranslate[1] = boxTranslatation[1];
    // posTranslate[2] = boxTranslatation[2];
  
    // tf::Quaternion inverse_zivid( 0.7054882, 0.6790058, 0.1507701, -0.1360367);
    // //회전역시 마찬가지로 변환을 해주어야 한다.
    // Eigen::Vector3f boxRotation;
    // tf::Quaternion boxratationTF(g_Pose.orientation.x,g_Pose.orientation.y,g_Pose.orientation.z,g_Pose.orientation.w);

    // //먼저한 linear transformation이 뒤로온다.
    // boxratationTF = boxratationTF*inverse_zivid;

    // tf::Matrix3x3 boxratationTFMat(boxratationTF);
    // double roll,pitch,yaw;
    // boxratationTFMat.getRPY(roll,pitch,yaw);
    // boxRotation[0]=roll;
    // boxRotation[1]=pitch;
    // boxRotation[2]=yaw;

    // pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    // boxFilter.setInputCloud(cloud_msg);
    // boxFilter.setMin(minPoint);
    // boxFilter.setMax(maxPoint);
    // boxFilter.setTranslation(posTranslate);
    // boxFilter.setRotation(boxRotation);
    // boxFilter.filter(*cloud);

    // //voxel Filter
    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud);
    // sor.setLeafSize(0.04, 0.04, 0.04);
    // sor.filter(*cloud);
    // std::cout << "maxboxcor " << sor.getMaxBoxCoordinates() << std::endl;
    // std::cout << "minboxcor " << sor.getMinBoxCoordinates() << std::endl;
    // std::cout << "size: " << sor.getMaxBoxCoordinates() - sor.getMinBoxCoordinates() << std::endl;
    

    // //StatisticalOutlierRemoval Filter
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> rmv;
    // rmv.setInputCloud(cloud);
    // rmv.setMeanK(50);
    // rmv.setStddevMulThresh(1.0);
    // rmv.filter(*cloud);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filling (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointXYZRGB pointd;
    // pointd.x = posTranslate[0];
    // pointd.y = posTranslate[1];           
    // pointd.z = posTranslate[2];
    
    // cloud_filling->push_back(pointd);

    // ROS_INFO("request:Position       x=%f, y=%f, z=%f",g_Pose.position.x,g_Pose.position.y,g_Pose.position.z);
    // *cloud += *cloud_filling;

    // copyPointCloud(*cloud, *g_cloud_filtered);

    // Publish points
    sensor_msgs::PointCloud2 filtered_publish;

    pcl::toROSMsg(*restored_cloud,filtered_publish);
    filtered_publish.header = g_cloud.header;    //.header.frame_id = "base";// --> base header
    g_pub_filtered.publish(filtered_publish);
}

void rcv_cloud (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    ROS_INFO("rcv cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_msg,cloud_publish);
    cloud_publish.header = msg->header;  //-->zvid_pc_50704

    //std::cerr<<"frame id : "<<cloud_publish.header.frame_id<<std::endl;
    if(cloud_publish.width * cloud_publish.height > 1)
    {
        g_cloud = cloud_publish;
    }

    //echo publish clouds
    g_pub_clone.publish(g_cloud);
    
}

bool extractRoi(assembly_msgs::GetOccupancyVoxel::Request &req,
    assembly_msgs::GetOccupancyVoxel::Response &res)
{
    ROS_INFO("Rececivecd !!");
    g_Pose = req.pose;
    cloud_cb();
    // sensor_msgs::PointCloud2 responese_cloud;
    // pcl::toROSMsg(*g_cloud_filtered,responese_cloud);
    // res.PointCloud2 = responese_cloud;  
    // res.respose = req.pose;

    //이거 써야지 제대로 됨
    return true;
}
*/

void rcv_cloud (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    //ROS_INFO("rcv cloud");
    if(!smapore)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg,*cloud_msg);
        sensor_msgs::PointCloud2 cloud_publish;


        //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        //sor.setInputCloud(cloud_msg);
        //sor.setLeafSize(0.02, 0.02, 0.02);
        //sor.filter(*cloud_msg);

        pcl::toROSMsg(*cloud_msg,cloud_publish);
        cloud_publish.header = msg->header;  //-->zvid_pc_50704

        //std::cerr<<"frame id : "<<cloud_publish.header.frame_id<<std::endl;
        if(cloud_publish.width * cloud_publish.height > 1)
        {
            g_cloud = cloud_publish;
        }
    }

    //echo publish clouds
    //g_pub_clone.publish(g_cloud);
    
}

void roiCallback (const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr& boxes)
{
    //Eigen::Affine3f T_KO,T_LK,T_LO;
    smapore = true;
    Eigen::Affine3f T_LK;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(unsigned int num = 0; num < boxes->bounding_boxes.size(); num++)
    {
        //T_KO.translation() << (boxes->bounding_boxes.at(num).xmin+boxes->bounding_boxes.at(num).xmax)/2.0, (boxes->bounding_boxes.at(num).ymin+boxes->bounding_boxes.at(num).ymax)/2.0
        //, (boxes->bounding_boxes.at(num).zmin+boxes->bounding_boxes.at(num).zmax)/2.0;
        //T_KO.linear() = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();

        T_LK.translation() << -0.00014467 , 0.014879, 0.000155564;
        T_LK.linear() = Eigen::Quaternionf(-0.49997,0.49875,-0.49919,0.50208).toRotationMatrix();
        //std::cerr<<"Max Points  num :"<<num<<"  , "<<maxPoint<<std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rcv (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);       
        pcl::fromROSMsg(g_cloud,*cloud_filtered);
        //pcl::transformPointCloud (*cloud_rcv, *cloud_filtered, T_LK.inverse());

        Eigen::Vector4f minPoint;
        minPoint[0] = boxes->bounding_boxes.at(num).xmin;
        minPoint[1] = boxes->bounding_boxes.at(num).ymin;
        minPoint[2] = boxes->bounding_boxes.at(num).zmin;
        minPoint[3] = 1.0;
        Eigen::Vector4f maxPoint;
        maxPoint[0] = boxes->bounding_boxes.at(num).xmax;
        maxPoint[1] = boxes->bounding_boxes.at(num).ymax;
        maxPoint[2] = boxes->bounding_boxes.at(num).zmax;
        maxPoint[3] = 1.0;

        Eigen::Vector3f posTranslate;
        posTranslate[0] = (boxes->bounding_boxes.at(num).xmin+boxes->bounding_boxes.at(num).xmax )/2;
        posTranslate[1] = (boxes->bounding_boxes.at(num).ymin+boxes->bounding_boxes.at(num).ymax )/2;
        posTranslate[2] = (boxes->bounding_boxes.at(num).zmin+boxes->bounding_boxes.at(num).zmax )/2;     
        //Eigen::Vector3f boxRotation;
        //posTranslate.setZero();
        //boxRotation.setZero();

        sensor_msgs::PointCloud2 debug;
        pcl::toROSMsg(*cloud_filtered,debug);
        g_pub_test.publish(debug);

        pcl::PointXYZRGB point;
        //for(unsigned int i = 0; i < cloud_filtered->points.size();i++)
       // {
        //    point.x = cloud_filtered->points.at(i).x;
        //    point.y = cloud_filtered->points.at(i).y;
        //    point.z = cloud_filtered->points.at(i).z;

        //    if( !( (point.x < minPoint[0] || point.y < minPoint[1] || point.z < minPoint[2]) ||(point.x > maxPoint[0] || point.y > maxPoint[1] || point.z > maxPoint[2]) ) )
        //    {
        //        cloud_rcv->header = cloud_filtered->header;
        //        cloud_rcv->push_back(cloud_filtered->points.at(i));
        //    }
            //std::cerr<<"point :"<<point<<std::endl;
            //std::cerr<<"max point :"<<maxPoint<<std::endl;
            //std::cerr<<"min point :"<<minPoint<<std::endl;
        //}

        pcl::CropBox<pcl::PointXYZRGB> boxFilter;
        //boxFilter.setKeepOrganized(false);
        //boxFilter.setNegative(false);
        boxFilter.setMin(minPoint);
        boxFilter.setMax(maxPoint);
        boxFilter.setInputCloud(cloud_filtered);
        boxFilter.setTransform(T_LK);
        //boxFilter.setRotation(boxRotation);
        boxFilter.filter(*cloud_rcv);
        cloud_rcv->header = cloud_filtered->header;


        //for Debug
        point.x = boxes->bounding_boxes.at(num).xmin;
        point.y = boxes->bounding_boxes.at(num).ymin;
        point.z = boxes->bounding_boxes.at(num).zmin;
        point.r = 255;
        point.b = 0;
        point.g = 0;
        cloud_rcv->push_back(point);

        point.x = boxes->bounding_boxes.at(num).xmax;
        point.y = boxes->bounding_boxes.at(num).ymax;
        point.z = boxes->bounding_boxes.at(num).zmax;
        point.r = 0;
        point.b = 255;
        point.g = 0;
        cloud_rcv->push_back(point);

        *cloud  = *cloud + *cloud_rcv;
    }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud,cloud_publish);
    cloud_publish.header = g_cloud.header;
    g_pub_filtered.publish(cloud_publish);
    smapore = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_sgm_node");
    ros::NodeHandle nh;  

     // Create a ROS subscriber for the input point cloud
    // when you scrible some topic you should include "/"
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, rcv_cloud); //camera/depts/points
    ros::Subscriber sub_box = nh.subscribe ("/darknet_ros_3d/bounding_boxes", 1, roiCallback);
    

    // Create a ROS publisher for the output point cloud
    //g_pub_clone = nh.advertise<sensor_msgs::PointCloud2> ("kinect/PointCloud", 1);
    //pub_equ = nh.advertise<pcl_msgs::ModelCoefficients> ("Plane/Coefficient", 1);
    g_pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/ROI/FilteredCloud", 1);
    g_pub_test = nh.advertise<sensor_msgs::PointCloud2>("/DEBUG/FilteredCloud", 1);

    // Create a Ros Srv for plane Equation ax+by+cz+d = 0
    //ros::ServiceServer service = nh.advertiseService("/roiExtractor", extractRoi);
    //spin
    ros::spin();
}