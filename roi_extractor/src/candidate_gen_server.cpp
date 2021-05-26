#include <ros/ros.h>

//PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

// System
#include <sstream>
#include <string>
#include <vector>


ros::Publisher seg_cloud_pub;

void rcv_cloud (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert to pcl point cloud
    //ROS_INFO("rcv cloud");
    //search Method
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg,*cloud_msg);
    sensor_msgs::PointCloud2 cloud_publish;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_msg);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud_msg);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(0.5 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.5);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j= 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
                pcl::PointXYZRGB point;
                point.x = cloud_msg->points[*pit].x;
                point.y = cloud_msg->points[*pit].y;
                point.z = cloud_msg->points[*pit].z;
                if (j == 0) //Red	#FF0000	(255,0,0)  (윗 문)
			     {
				      point.r = 0;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 1) //Lime	#00FF00	(0,255,0)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 2) // Blue	#0000FF	(0,0,255) (아랫문)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 0;
			     }
                 colored_cloud->push_back(point);
        }
         j++;
    }
    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

    pcl::toROSMsg(*colored_cloud,cloud_publish);
    cloud_publish.header = msg->header;  //-->zvid_pc_50704

    //std::cerr<<"frame id : "<<cloud_publish.header.frame_id<<std::endl;
    if(cloud_publish.width * cloud_publish.height > 1)
    {
        seg_cloud_pub.publish(cloud_publish);
        //g_cloud = cloud_publish;
    }

    //echo publish clouds
    //g_pub_clone.publish(g_cloud);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roi_plane_segmentation");
    ros::NodeHandle nh;  

    //init robot Parameter
    double finger_length_x = 0.02;
    double finger_length_y = 0.02;
    double finger_length_z = 0.05;
    double init_bite  = 0.15;

    bool voxelize = true;
    bool remove_outliers = false;

    //std::vector<double> workspace = stringToDouble(workspace_str);
    //std::vector<double> camera_pose 

    ros::Subscriber sub = nh.subscribe ("/ROI/FilteredCloud", 1, rcv_cloud); //camera/depts/points
    
    seg_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented/plane/cloud", 1);

    // Create a Ros Srv for plane Equation ax+by+cz+d = 0
    //ros::ServiceServer service = nh.advertiseService("/roiExtractor", extractRoi);
    //spin
    ros::spin();
}


//plane Extract 
/*
    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud(cloud_msg);
    //sor.setLeafSize(0.02, 0.02, 0.02);
    //sor.filter(*cloud_msg);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);

    seg.setInputCloud (cloud_msg);
    seg.segment (*inliers, *coefficients);

    for (const auto& idx: inliers->indices)
    {
       cloud_msg->points[idx].r =255;
       cloud_msg->points[idx].g =0;
       cloud_msg->points[idx].b =0;
    }

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inlier (new pcl::PointCloud<pcl::PointXYZRGB>);
    // int original_size(cloud_msg->height*cloud_msg->width);
    // int n_planes(0);

    //  while (cloud_msg->height*cloud_msg->width>original_size*70/100){

    //     // Fit a plane
    //     seg.setInputCloud(cloud_msg);
    //     seg.segment(*inliers, *coefficients);

    //     // Check result
    //     if (inliers->indices.size() == 0)
    //         break;

    //     for (int i=0;i<inliers->indices.size();i++){
    //         // Get Point
    //         pcl::PointXYZRGB pt = cloud_msg->points[inliers->indices[i]];

    //         // Copy point to noew cloud
    //         pcl::PointXYZRGB pt_color;
    //         pt_color.x = pt.x;
    //         pt_color.y = pt.y;
    //         pt_color.z = pt.z;
    //         if(i==0 && n_planes == 0)
    //         {
    //             pt_color.r = 255;
    //             pt_color.g = 0;
    //             pt_color.b = 0;
    //         }
    //         else
    //         {
    //             pt_color.r = 0;
    //             pt_color.g = 255;
    //             pt_color.b = 0;
    //         }
    //         cloud_inlier->points.push_back(pt_color);
    //     }

    //     // Extract inliers
    //     extract.setInputCloud(cloud_msg);
    //     extract.setIndices(inliers);
    //     extract.setNegative(true);
    //     pcl::PointCloud<pcl::PointXYZRGB> cloudF;
    //     extract.filter(cloudF);
    //     cloud_msg->swap(cloudF);

    //     // Display infor
    //     // ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
    //     //          _name.c_str(),n_planes,
    //     //          coefficients->values[0],(coefficients->values[1]>=0?"+":""),
    //     //          coefficients->values[1],(coefficients->values[2]>=0?"+":""),
    //     //          coefficients->values[2],(coefficients->values[3]>=0?"+":""),
    //     //          coefficients->values[3],
    //     //          inliers->indices.size(),original_size);
    //     // ROS_INFO("%s: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",_name.c_str(),mean_error,sigma,max_error);
    //     // ROS_INFO("%s: poitns left in cloud %i",_name.c_str(),cloud->width*cloud->height);

    //     // Nest iteration
    //     n_planes++;
    // }
    */


    //////////////////////////// region growing
    // pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    // normal_estimator.setSearchMethod(tree);
    // normal_estimator.setInputCloud(cloud_msg);
    // normal_estimator.setKSearch(50);
    // normal_estimator.compute(*normals);

    // pcl::IndicesPtr indices (new std::vector <int>);
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud(cloud_msg);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 2.0);
    // pass.filter(*indices);

    // pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    // reg.setMinClusterSize(10);
    // reg.setMaxClusterSize(1000000);
    // reg.setSearchMethod(tree);
    // reg.setNumberOfNeighbours(30);
    // reg.setInputCloud(cloud_msg);
    // //reg.setIndices (indices);
    // reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    // reg.setCurvatureThreshold(1.0);

    // std::vector<pcl::PointIndices> clusters;
    // reg.extract(clusters);


    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();