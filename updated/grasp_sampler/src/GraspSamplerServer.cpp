#include "ros/ros.h"

//custum msgs
#include "task_assembly/door_open_planner.h"

#include "../include/grasp_sampler/arm_interface.h"

// bool add(task_assembly::door_open_planner::Request  &req,
//          task_assembly::door_open_planner::Response &res)
// {
//   std::cout<<"hello "<<std::endl;
//   return true;
// }

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "GraspSamplerClient");
  // ros::NodeHandle n;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  srand(spec.tv_nsec);

  ros::init(argc, argv, "GraspSamplerServer");
  ros::NodeHandle nh("~");

  // load urdf path
  std::string yaml_path;
  nh.getParam("yaml_path", yaml_path);

  YAMLConfig config;
  config.loadConfig(yaml_path);

  //ros::ServiceServer service = nh.advertiseService("/plane_door_grasp_motion", add);

  arm_controller_interface panda_door_planner(nh,config,100.0);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}