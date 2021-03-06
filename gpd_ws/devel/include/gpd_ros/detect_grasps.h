// Generated by gencpp from file gpd_ros/detect_grasps.msg
// DO NOT EDIT!


#ifndef GPD_ROS_MESSAGE_DETECT_GRASPS_H
#define GPD_ROS_MESSAGE_DETECT_GRASPS_H

#include <ros/service_traits.h>


#include <gpd_ros/detect_graspsRequest.h>
#include <gpd_ros/detect_graspsResponse.h>


namespace gpd_ros
{

struct detect_grasps
{

typedef detect_graspsRequest Request;
typedef detect_graspsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct detect_grasps
} // namespace gpd_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gpd_ros::detect_grasps > {
  static const char* value()
  {
    return "6544a7e3669d79f6069fe9d81fee1c1e";
  }

  static const char* value(const ::gpd_ros::detect_grasps&) { return value(); }
};

template<>
struct DataType< ::gpd_ros::detect_grasps > {
  static const char* value()
  {
    return "gpd_ros/detect_grasps";
  }

  static const char* value(const ::gpd_ros::detect_grasps&) { return value(); }
};


// service_traits::MD5Sum< ::gpd_ros::detect_graspsRequest> should match
// service_traits::MD5Sum< ::gpd_ros::detect_grasps >
template<>
struct MD5Sum< ::gpd_ros::detect_graspsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gpd_ros::detect_grasps >::value();
  }
  static const char* value(const ::gpd_ros::detect_graspsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gpd_ros::detect_graspsRequest> should match
// service_traits::DataType< ::gpd_ros::detect_grasps >
template<>
struct DataType< ::gpd_ros::detect_graspsRequest>
{
  static const char* value()
  {
    return DataType< ::gpd_ros::detect_grasps >::value();
  }
  static const char* value(const ::gpd_ros::detect_graspsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gpd_ros::detect_graspsResponse> should match
// service_traits::MD5Sum< ::gpd_ros::detect_grasps >
template<>
struct MD5Sum< ::gpd_ros::detect_graspsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gpd_ros::detect_grasps >::value();
  }
  static const char* value(const ::gpd_ros::detect_graspsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gpd_ros::detect_graspsResponse> should match
// service_traits::DataType< ::gpd_ros::detect_grasps >
template<>
struct DataType< ::gpd_ros::detect_graspsResponse>
{
  static const char* value()
  {
    return DataType< ::gpd_ros::detect_grasps >::value();
  }
  static const char* value(const ::gpd_ros::detect_graspsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GPD_ROS_MESSAGE_DETECT_GRASPS_H
