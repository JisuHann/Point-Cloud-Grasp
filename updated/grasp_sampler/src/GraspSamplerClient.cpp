#include <ros/ros.h>
#include <cstdlib>

// custom srv
#include "task_assembly/door_open_planner.h"

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

    return 0;
}