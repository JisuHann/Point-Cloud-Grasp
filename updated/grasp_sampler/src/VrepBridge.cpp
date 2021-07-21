#include "../include/grasp_sampler/VrepBridge.h"

sim_controller_interface::sim_controller_interface(ros::NodeHandle &_nh,double _hz):
rate_(_hz)
{

    // publisher
    // vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
    // vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
    // vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    // vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);

    // SubScrible
}