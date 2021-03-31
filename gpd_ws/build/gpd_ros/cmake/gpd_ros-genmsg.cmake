# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gpd_ros: 6 messages, 1 services")

set(MSG_I_FLAGS "-Igpd_ros:/home/min/gpd_ws/src/gpd_ros/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gpd_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" "geometry_msgs/Vector3:std_msgs/Float32:geometry_msgs/Point:gpd_ros/GraspConfig:std_msgs/Header"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" "gpd_ros/CloudSources:sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header:geometry_msgs/Point:std_msgs/Int64"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Int64:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" "std_msgs/Float32:geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" "gpd_ros/CloudSources:sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header:geometry_msgs/Point:std_msgs/Int64"
)

get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_custom_target(_gpd_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gpd_ros" "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" "gpd_ros/CloudSources:std_msgs/Float32:sensor_msgs/PointCloud2:sensor_msgs/PointField:gpd_ros/GraspConfig:geometry_msgs/Vector3:gpd_ros/GraspConfigList:std_msgs/Header:gpd_ros/CloudIndexed:geometry_msgs/Point:std_msgs/Int64"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)
_generate_msg_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)

### Generating Services
_generate_srv_cpp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
)

### Generating Module File
_generate_module_cpp(gpd_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gpd_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gpd_ros_generate_messages gpd_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_dependencies(gpd_ros_generate_messages_cpp _gpd_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gpd_ros_gencpp)
add_dependencies(gpd_ros_gencpp gpd_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gpd_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)
_generate_msg_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)

### Generating Services
_generate_srv_eus(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
)

### Generating Module File
_generate_module_eus(gpd_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gpd_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gpd_ros_generate_messages gpd_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_dependencies(gpd_ros_generate_messages_eus _gpd_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gpd_ros_geneus)
add_dependencies(gpd_ros_geneus gpd_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gpd_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)
_generate_msg_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)

### Generating Services
_generate_srv_lisp(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
)

### Generating Module File
_generate_module_lisp(gpd_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gpd_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gpd_ros_generate_messages gpd_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_dependencies(gpd_ros_generate_messages_lisp _gpd_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gpd_ros_genlisp)
add_dependencies(gpd_ros_genlisp gpd_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gpd_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)
_generate_msg_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)

### Generating Services
_generate_srv_nodejs(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
)

### Generating Module File
_generate_module_nodejs(gpd_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gpd_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gpd_ros_generate_messages gpd_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_dependencies(gpd_ros_generate_messages_nodejs _gpd_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gpd_ros_gennodejs)
add_dependencies(gpd_ros_gennodejs gpd_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gpd_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)
_generate_msg_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)

### Generating Services
_generate_srv_py(gpd_ros
  "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv"
  "${MSG_I_FLAGS}"
  "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
)

### Generating Module File
_generate_module_py(gpd_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gpd_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gpd_ros_generate_messages gpd_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfigList.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudIndexed.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSources.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/SamplesMsg.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/GraspConfig.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/msg/CloudSamples.msg" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/gpd_ws/src/gpd_ros/srv/detect_grasps.srv" NAME_WE)
add_dependencies(gpd_ros_generate_messages_py _gpd_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gpd_ros_genpy)
add_dependencies(gpd_ros_genpy gpd_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gpd_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gpd_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(gpd_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(gpd_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gpd_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gpd_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(gpd_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(gpd_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gpd_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gpd_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(gpd_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(gpd_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gpd_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gpd_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(gpd_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(gpd_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gpd_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gpd_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(gpd_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(gpd_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gpd_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
