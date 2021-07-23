#include <ros/ros.h>
#include <cstdlib>

// custom srv
#include "task_assembly/door_open_planner.h"

// msgs
#include <geometry_msgs/Pose.h>

const std::string Joint_name[7] =
    {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6",
        "panda_joint7"};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GraspSamplerClient");
    ros::NodeHandle nh("~");

    ros::ServiceClient ros_grasp_sampler_client = nh.serviceClient<task_assembly::door_open_planner>("/plane_door_grasp_motion");
    task_assembly::door_open_planner grasp_sampler_srv;

    if(ros::ok()) 
        ros::spinOnce();

    // Joint State   ---> 7-DOF franka panda
    sensor_msgs::JointState q_service;

    q_service.name.resize(7);
    q_service.position.resize(7);
    q_service.velocity.resize(7);
    q_service.effort.resize(7);

    for (int i = 0; i < 7; i++)
    {
        q_service.name[i] = Joint_name[i];
        q_service.position[i] = 0.1;
        q_service.velocity[i] = 0.0;
        q_service.effort[i] = 0.0;
    }

    q_service.position[0] = 0.0;
    q_service.position[1] = 0.0;
    q_service.position[2] = 0.0;
    q_service.position[3] = 0.0;
    q_service.position[4] = 0.0;
    q_service.position[5] = 0.0;
    q_service.position[6] = 0.0;

    grasp_sampler_srv.request.current_arm_joint_state = q_service;
    grasp_sampler_srv.request.interpolate_path.data = true;

    if (ros_grasp_sampler_client.call(grasp_sampler_srv)) // call the service and return the response value
    {
        std::cout<<"service call success !!"<<std::endl;
        //ROS_INFO("send srv, srv.Request.a and b : %1d, %1d", (long int)srv.request.a, (long int)srv.request.b);
        //ROS_INFO("recieve srv, srv.Response.result : %1d", (long int)srv.response.result);
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    /// after processing
    // geometry_msgs::Pose result_ = grasp_sampler_srv.response.grasp_pose;

    // tf::TransformListener listener;
    // tf::StampedTransform transform1;
    // tf::StampedTransform transform2;
    // tf::StampedTransform transform3;

    // tf::TransformBroadcaster tf_broadcaster;
    // geometry_msgs::TransformStamped tf_trans_1;
    // geometry_msgs::TransformStamped tf_trans_2;
    // geometry_msgs::TransformStamped tf_trans_3;
    // geometry_msgs::TransformStamped tf_trans_4;

    // Eigen::Vector3d door_pos;
    // door_pos.setZero();
    // ros::Time current_time;
    // ros::Rate rate(100.0);

    // double doorAngle = 0.0;
    // double doorOri = 0.0;

    // while (ros::ok())
    // {
    //     // rb.BaseTransform();
    //     // rb.ReadQRCode();
    //     // rb.WriteAngle(doorangle);
    //     // rb.WriteDoorPosition(door_pos);
    //     ros::Time current_time;
    //     current_time = ros::Time::now();

    //     tf_trans_1.header.stamp = current_time;
    //     tf_trans_1.header.frame_id = "base";
    //     tf_trans_1.child_frame_id = "camera";
    //     tf_trans_1.transform.translation.x = -0.27+0.027;
    //     tf_trans_1.transform.translation.y = -0.205;
    //     tf_trans_1.transform.translation.z = 1.54+0.015;
    //     tf_trans_1.transform.rotation.w = 0.5;
    //     tf_trans_1.transform.rotation.x = -0.5;
    //     tf_trans_1.transform.rotation.y = 0.5;
    //     tf_trans_1.transform.rotation.z = -0.5;

    //     tf_trans_2.header.stamp = current_time;
    //     tf_trans_2.header.frame_id = "camera";
    //     tf_trans_2.child_frame_id = "kinect2_rgb_optical_frame";
    //     tf_trans_2.transform.translation.x = 0.0;
    //     tf_trans_2.transform.translation.y = 0.0;
    //     tf_trans_2.transform.translation.z = 0.0;
    //     tf_trans_2.transform.rotation.w =  0.991;
    //     tf_trans_2.transform.rotation.x = -0.131;
    //     tf_trans_2.transform.rotation.y = 0.0;
    //     tf_trans_2.transform.rotation.z = 0.0;
    //     /*
    //     tf_trans_3.header.stamp = current_time;
    //     tf_trans_3.header.frame_id = "kinect2/hd_fid_5";
    //     tf_trans_3.child_frame_id = "door_handle";
    //     tf_trans_3.transform.translation.x = -0.25;//-0.235+0.0382;
    //     tf_trans_3.transform.translation.y = 0;
    //     tf_trans_3.transform.translation.z = 0.153;//0.175-0.0118;
    //     tf_trans_3.transform.rotation.w = 1;
    //     tf_trans_3.transform.rotation.x = 0.0;
    //     tf_trans_3.transform.rotation.y = 0.0;
    //     tf_trans_3.transform.rotation.z = 0.0;
    //     */
    //     tf_trans_4.header.stamp = current_time;
    //     tf_trans_4.header.frame_id = "kinect2/hd_fid_8";
    //     tf_trans_4.child_frame_id = "origin";
    //     tf_trans_4.transform.translation.x = 1.5;
    //     tf_trans_4.transform.translation.y = -1.430;
    //     tf_trans_4.transform.translation.z = 2.1;
    //     tf_trans_4.transform.rotation.w = 0.5;
    //     tf_trans_4.transform.rotation.x = -0.5;
    //     tf_trans_4.transform.rotation.y = 0.5;
    //     tf_trans_4.transform.rotation.z = 0.5;
        
    //     tf_broadcaster.sendTransform(tf_trans_1);
    //     tf_broadcaster.sendTransform(tf_trans_2);
    //     // tf_broadcaster.sendTransform(tf_trans_3);
    //     tf_broadcaster.sendTransform(tf_trans_4);

    //     // try{
    //     // listener.lookupTransform("/origin", "/base", ros::Time(0), transform1);
    //     // }
    //     // catch (tf::TransformException ex){
    //     // ROS_ERROR("%s",ex.what());
    //     // ros::Duration(0.1).sleep();
    //     // }
        
    //     try{
    //     listener.lookupTransform("/kinect2/hd_fid_8", "/kinect2/hd_fid_5", ros::Time(0), transform2);
    //     }
    //     catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(0.1).sleep();
    //     }
    //     try{
    //     listener.lookupTransform("/base", "/kinect2/hd_fid_8", ros::Time(0), transform3);
    //     }
    //     catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(0.1).sleep();
    //     }

    //     Eigen::Quaterniond q1,q2;
    //     Eigen::Matrix3d R1, R2, R3;
    //     Eigen::Vector3d Euler; 
    //     // q1.x() = transform2.getRotation().x();
    //     // q1.y() = transform2.getRotation().y();
    //     // q1.z() = transform2.getRotation().z();
    //     // q1.w() = transform2.getRotation().w();
    //     // R1 = q1.normalized().toRotationMatrix();
    //     // Euler = R1.eulerAngles(0, 1, 2);
    //     // // doorangle = Euler(1);
    //     // doorangle = normAngle(Euler(1), -M_PI/2.0);
    //     // doorangle = -doorangle + 38*M_PI/180 ;

    //     q1.x() = transform2.getRotation().x();
    //     q1.y() = transform2.getRotation().y();
    //     q1.z() = transform2.getRotation().z();
    //     q1.w() = transform2.getRotation().w();
    //     R1 = q1.normalized().toRotationMatrix();
    //     Euler = R1.eulerAngles(0, 1, 2);
    //     // doorangle = Euler(1);
    //     doorangle = asin(R1(0, 2));
    //     doorangle = doorangle + 45.0 * M_PI / 180.0;



    //     q2.x() = transform3.getRotation().x();
    //     q2.y() = transform3.getRotation().y();
    //     q2.z() = transform3.getRotation().z();
    //     q2.w() = transform3.getRotation().w();
    //     R2 = q2.normalized().toRotationMatrix();        
    //     doorori =  asin(R2(1, 2));

    //     door_pos(0) = transform3.getOrigin().x();
    //     door_pos(1) = transform3.getOrigin().y();
    //     door_pos(2) = doorori;

    //     //std::cout << R2 << "\n" << std::endl;

    //     //std::cout << transform1.getOrigin().x() << "\t" << transform1.getOrigin().y() << "\t" << transform1.getOrigin().z() <<"\n";
    //    // std::cout << doorangle*180/M_PI <<"\n";

    // }


    return 0;
}