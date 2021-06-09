#include "../include/roi_extractor/handle_gen.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh("~");

    // std::string yaml_path;
    // nh.getParam("yaml_path", yaml_path);

    // YAMLConfig config;
    // config.loadConfig(yaml_path)

    handle_sampler handle_sampler_interface(nh,10);
    
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}