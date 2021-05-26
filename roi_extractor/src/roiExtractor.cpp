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

void rcv_cloud (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    //ROS_INFO("rcv cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);


    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud(cloud_msg);
    //sor.setLeafSize(0.02, 0.02, 0.02);
    //sor.filter(*cloud_msg);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_msg,cloud_publish);
    cloud_publish.header = msg->header;  //-->zvid_pc_50704

    //std::cerr<<"frame id : "<<cloud_publish.header.frame_id<<std::endl;
    if(cloud_publish.width * cloud_publish.height > 1)
    {
        g_cloud = cloud_publish;
    }
    
}

void roiCallback (const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr& boxes)
{
    //Eigen::Affine3f T_KO,T_LK,T_LO;
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
        //boxFilter.setTransform(T_LK);
        //boxFilter.setRotation(boxRotation);
        boxFilter.filter(*cloud_rcv);
        cloud_rcv->header = cloud_filtered->header;


        //for Debug
        // point.x = boxes->bounding_boxes.at(num).xmin;
        // point.y = boxes->bounding_boxes.at(num).ymin;
        // point.z = boxes->bounding_boxes.at(num).zmin;
        // point.r = 255;
        // point.b = 0;
        // point.g = 0;
        // cloud_rcv->push_back(point);

        // point.x = boxes->bounding_boxes.at(num).xmax;
        // point.y = boxes->bounding_boxes.at(num).ymax;
        // point.z = boxes->bounding_boxes.at(num).zmax;
        // point.r = 0;
        // point.b = 255;
        // point.g = 0;
        // cloud_rcv->push_back(point);

        *cloud  = *cloud + *cloud_rcv;
    }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud,cloud_publish);
    cloud_publish.header = g_cloud.header;
    g_pub_filtered.publish(cloud_publish);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "n_sgm_node");
    ros::NodeHandle nh;  

     // Create a ROS subscriber for the input point cloud
    // when you scrible some topic you should include "/"
    ros::Subscriber sub = nh.subscribe ("/sim/vrep/points", 1, rcv_cloud); //camera/depts/points
    ros::Subscriber sub_box = nh.subscribe ("/darknet_ros_3d/bounding_boxes", 1, roiCallback);
    

    // Create a ROS publisher for the output point cloud
    //g_pub_clone = nh.advertise<sensor_msgs::PointCloud2> ("kinect/PointCloud", 1);
    //pub_equ = nh.advertise<pcl_msgs::ModelCoefficients> ("Plane/Coefficient", 1);
    g_pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/ROI/FilteredCloud", 1);
    g_pub_test = nh.advertise<sensor_msgs::PointCloud2>("/DEBUG/FilteredCloud", 1);

    // Create a Ros Srv for plane Equation ax+by+cz+d = 0
    ros::spin();
}