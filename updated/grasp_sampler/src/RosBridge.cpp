#include "../include/grasp_sampler/RosBridge.h"

RosBridge::RosBridge(ros::NodeHandle nh_, double hz_) : rate_(hz_)
{
    qr_transform_sub_ = nh_.subscribe("/kinect2/hd/fiducial_transforms", 100, &RosBridge::qrtransformCallback, this);
    door_angle_pub_ = nh_.advertise<std_msgs::Float64>("/door_angle", 100);
    door_pos_pub_ = nh_.advertise<geometry_msgs::Point>("/door_pos",100);
    //  BaseTransform();

}
RosBridge::~RosBridge()
{

}

void RosBridge::qrtransformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
{
    //std::cout << msg->transforms.size() <<std::endl;
    //std::cout << msg->transforms[0].transform.rotation.x <<std::endl;
    
    // TODO : handling the situation when there is "absent" data from Kinect
    // TODO : handling the situation when there is "absent" data from Kinect

    
    // std::cout <<"\n";
    // std::cout << msg->transforms[0].fiducial_id <<"\t" <<msg->transforms[1].fiducial_id<<"\n";
    // Eigen::Vector3d x1, x2;

    // if (msg->transforms[0].fiducial_id == 0)
    // {
    //     q1.x() = msg->transforms[1].transform.rotation.x;
    //     q1.y() = msg->transforms[1].transform.rotation.y;
    //     q1.z() = msg->transforms[1].transform.rotation.z;
    //     q1.w() = msg->transforms[1].transform.rotation.w;

    //     q2.x() = msg->transforms[0].transform.rotation.x;
    //     q2.y() = msg->transforms[0].transform.rotation.y;
    //     q2.z() = msg->transforms[0].transform.rotation.z;
    //     q2.w() = msg->transforms[0].transform.rotation.w;

    //     x1(1) = -msg->transforms[1].transform.translation.x;
    //     x1(2) = -msg->transforms[1].transform.translation.y;
    //     x1(0) =  msg->transforms[1].transform.translation.z;

    //     x2(1) = -msg->transforms[0].transform.translation.x;
    //     x2(2) = -msg->transforms[0].transform.translation.y;
    //     x2(0) =  msg->transforms[0].transform.translation.z;
    // }
    // else if (msg->transforms[0].fiducial_id == 1)
    // {
    //     q1.x() = msg->transforms[0].transform.rotation.x;
    //     q1.y() = msg->transforms[0].transform.rotation.y;
    //     q1.z() = msg->transforms[0].transform.rotation.z;
    //     q1.w() = msg->transforms[0].transform.rotation.w;

    //     q2.x() = msg->transforms[1].transform.rotation.x;
    //     q2.y() = msg->transforms[1].transform.rotation.y;
    //     q2.z() = msg->transforms[1].transform.rotation.z;
    //     q2.w() = msg->transforms[1].transform.rotation.w;

    //     x1(1) = -msg->transforms[0].transform.translation.x;
    //     x1(2) = -msg->transforms[0].transform.translation.y;
    //     x1(0) =  msg->transforms[0].transform.translation.z;

    //     x2(1) = -msg->transforms[1].transform.translation.x;
    //     x2(2) = -msg->transforms[1].transform.translation.y;
    //     x2(0) =  msg->transforms[1].transform.translation.z;
    // }

    //     R1 = q1.normalized().toRotationMatrix();
    //     R2 = q2.normalized().toRotationMatrix();
    //     R3 = R1.transpose() * R2;
    //     Euler = R3.eulerAngles(0, 1, 2);
    //     doorangle_ = normAngle(Euler(1), -M_PI/2.0);

    //     Eigen::Vector3d Euler_R1;
    //     Euler_R1 = R1.eulerAngles(0, 1, 2);
    //     double qr1angle;
    //     qr1angle = normAngle(Euler_R1(1), -M_PI/2.0);
        
    //     Eigen::Vector3d camera_p_husky;
    //     camera_p_husky(0) = -0.241+0.05;//hukuyo+err
    //     camera_p_husky(1) = -0.2035+0.032-0.029;//bar/2+kinect-err
    //     camera_p_husky(2) =  1.525-0.046;

    //     Eigen::Vector3d handle_p_upper_qr;
    //     handle_p_upper_qr(0) = -(0.300+0.012+0.053);
    //     handle_p_upper_qr(1) = -0.04;
    //     handle_p_upper_qr(2) = 0.005;

    //     Eigen::Vector3d hinge_p_upper_qr;
    //     hinge_p_upper_qr(0) = 0.065;
    //     hinge_p_upper_qr(1) = 0;
    //     hinge_p_upper_qr(2) = -0.025;

    //     Eigen::Vector3d upper_qr_p_hinge;
    //     upper_qr_p_hinge(0) = -0.03057; 
    //     upper_qr_p_hinge(1) = 0.0625738;
    //     upper_qr_p_hinge(2) = 0;

    //     Eigen::Matrix3d camera_R_husky;
    //     double camera_angle = (17-1.09)*M_PI/180; // 17-body angle
    //     camera_R_husky << cos(camera_angle), 0, sin(camera_angle),
    //     0, 1, 0,
    //     -sin(camera_angle), 0, cos(camera_angle);

    //     Eigen::Matrix3d door_offset_R;
    //     double door_offset = 5*M_PI/180; 
    //     door_offset_R << cos(door_offset), 0, sin(door_offset),
    //     0,1,0, 
    //     -sin(door_offset), 0, cos(door_offset);

    //     Eigen::Matrix3d R0;
    //     R0 << 0, 0, 1,
    //     -1, 0, 0,
    //     0, -1, 0;

    //     Eigen::Matrix3d qr_R_husky;
    //     double door_a = qr1angle + 5*M_PI/180; 
    //     qr_R_husky << cos(door_a), -sin(door_a), 0, 
    //     sin(door_a), cos(door_a), 0, 
    //     0,0,1;
    
    //     Eigen::Vector3d x3, x4, x5, x6;
    //     x1 = camera_R_husky * x1 + camera_p_husky;
    //     x2 = camera_R_husky * x2 + camera_p_husky;
    //     x3 = camera_R_husky*(R0*R1.transpose())*handle_p_upper_qr + x1;
    //     x4 = camera_R_husky*(R0*R1.transpose())*hinge_p_upper_qr + x1;
    //     x5 = qr_R_husky * x1;
    //     x6 = upper_qr_p_hinge-x5;
    //     x6(2)=0;

       // std::cout <<"qr_upper_husky : " << x1.transpose() << "\n" <<"qr_lower_husky : "<< x2.transpose() << "\n";
       // std::cout <<"hd_upper_husky : " << x3.transpose() << "\n";
       // std::cout <<"hinge_husky    : " << x4.transpose() << "\n";
       // std::cout <<"qr_upper_hinge : " << x5.transpose() << "\n";
       // std::cout <<"husky_hinge    : " << x6.transpose() << "\n";  
        
     
}
void RosBridge::BaseTransform()
{

    
}

void RosBridge::ReadQRCode()
{
    ros::spinOnce();
}

void RosBridge::WriteAngle(double door_angle)
{
    

    doorangle.data = door_angle;
    door_angle_pub_.publish(doorangle);

    rate_.sleep();
}
void RosBridge::WriteDoorPosition(Eigen::Vector3d pos)
{
    

    doorpos.x = pos(0);
    doorpos.y = pos(1);
    doorpos.z = pos(2);
    door_pos_pub_.publish(doorpos);

    rate_.sleep();
}
