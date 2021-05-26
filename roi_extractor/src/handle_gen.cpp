#include "handle_gen.h"

handle_sampler::handle_sampler(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{
    yolo_detection_sub_ = _nh.subscribe("/darknet_ros_3d/bounding_boxes",10, &handle_sampler::reigon_cb, this)
}

void handle_sampler::reigon_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &_objpose)
{

}








// #include <ros/ros.h>

// //PCL includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>

// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
// #include <pcl/segmentation/region_growing_rgb.h>

// #include <iostream>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/filters/passthrough.h>

// // System
// #include <sstream>
// #include <string>
// #include <vector>


// ros::Publisher seg_cloud_pub;

// void rcv_cloud (const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     // Convert to pcl point cloud
//     //ROS_INFO("rcv cloud");
//     //search Method
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::fromROSMsg(*msg,*cloud_msg);
//     sensor_msgs::PointCloud2 cloud_publish;

//     pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//     pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

//     normal_estimator.setSearchMethod(tree);
//     normal_estimator.setInputCloud(cloud_msg);
//     normal_estimator.setKSearch(50);
//     normal_estimator.compute(*normals);

//     pcl::IndicesPtr indices (new std::vector <int>);
//     pcl::PassThrough<pcl::PointXYZRGB> pass;
//     pass.setInputCloud(cloud_msg);
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(0.0, 2.0);
//     pass.filter(*indices);

//     pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
//     reg.setMinClusterSize(50);
//     reg.setMaxClusterSize(1000000);
//     reg.setSearchMethod(tree);
//     reg.setNumberOfNeighbours(30);
//     reg.setInputCloud(cloud_msg);
//     //reg.setIndices (indices);
//     reg.setInputNormals(normals);
//     reg.setSmoothnessThreshold(0.5 / 180.0 * M_PI);
//     reg.setCurvatureThreshold(0.5);

//     std::vector<pcl::PointIndices> clusters;
//     reg.extract(clusters);

//     pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//     int j= 0;
//     for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
//     {
//         for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
//         {
//                 pcl::PointXYZRGB point;
//                 point.x = cloud_msg->points[*pit].x;
//                 point.y = cloud_msg->points[*pit].y;
//                 point.z = cloud_msg->points[*pit].z;
//                 if (j == 0) //Red	#FF0000	(255,0,0)  (윗 문)
// 			     {
// 				      point.r = 0;
// 				      point.g = 0;
// 				      point.b = 255;
// 			     }
// 			    else if (j == 1) //Lime	#00FF00	(0,255,0)
// 			     {
// 				      point.r = 0;
// 				      point.g = 255;
// 				      point.b = 0;
// 			     }
// 			    else if (j == 2) // Blue	#0000FF	(0,0,255) (아랫문)
// 			     {
// 				      point.r = 255;
// 				      point.g = 0;
// 				      point.b = 0;
// 			     }
//                  colored_cloud->push_back(point);
//         }
//          j++;
//     }
//     // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

//     pcl::toROSMsg(*colored_cloud,cloud_publish);
//     cloud_publish.header = msg->header;  //-->zvid_pc_50704

//     //std::cerr<<"frame id : "<<cloud_publish.header.frame_id<<std::endl;
//     if(cloud_publish.width * cloud_publish.height > 1)
//     {
//         seg_cloud_pub.publish(cloud_publish);
//         //g_cloud = cloud_publish;
//     }

//     //echo publish clouds
//     //g_pub_clone.publish(g_cloud);
    
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "roi_plane_segmentation");
//     ros::NodeHandle nh;  

//     //init robot Parameter
//     double finger_length_x = 0.02;
//     double finger_length_y = 0.02;
//     double finger_length_z = 0.05;
//     double init_bite  = 0.15;

//     bool voxelize = true;
//     bool remove_outliers = false;

//     //std::vector<double> workspace = stringToDouble(workspace_str);
//     //std::vector<double> camera_pose 

//     ros::Subscriber sub = nh.subscribe ("/ROI/FilteredCloud", 1, rcv_cloud); //camera/depts/points
    
//     seg_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented/plane/cloud", 1);

//     // Create a Ros Srv for plane Equation ax+by+cz+d = 0
//     //spin
//     ros::spin();
// }