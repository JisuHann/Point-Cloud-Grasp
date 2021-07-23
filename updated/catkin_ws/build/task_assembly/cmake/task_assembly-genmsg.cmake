# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "task_assembly: 3 messages, 1 services")

set(MSG_I_FLAGS "-Itask_assembly:/home/min/catkin_ws/src/task_assembly/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(task_assembly_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_custom_target(_task_assembly_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_assembly" "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" "task_assembly/BoundingBox3d:std_msgs/Header"
)

get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_custom_target(_task_assembly_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_assembly" "/home/min/catkin_ws/src/task_assembly/msg/target.msg" ""
)

get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_custom_target(_task_assembly_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_assembly" "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" "trajectory_msgs/JointTrajectoryPoint:trajectory_msgs/JointTrajectory:std_msgs/Bool:sensor_msgs/JointState:std_msgs/Header"
)

get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_custom_target(_task_assembly_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "task_assembly" "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg"
  "${MSG_I_FLAGS}"
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
)
_generate_msg_cpp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
)
_generate_msg_cpp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
)

### Generating Services
_generate_srv_cpp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
)

### Generating Module File
_generate_module_cpp(task_assembly
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(task_assembly_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(task_assembly_generate_messages task_assembly_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_cpp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_cpp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_dependencies(task_assembly_generate_messages_cpp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_cpp _task_assembly_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_assembly_gencpp)
add_dependencies(task_assembly_gencpp task_assembly_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_assembly_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg"
  "${MSG_I_FLAGS}"
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
)
_generate_msg_eus(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
)
_generate_msg_eus(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
)

### Generating Services
_generate_srv_eus(task_assembly
  "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
)

### Generating Module File
_generate_module_eus(task_assembly
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(task_assembly_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(task_assembly_generate_messages task_assembly_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_eus _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_eus _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_dependencies(task_assembly_generate_messages_eus _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_eus _task_assembly_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_assembly_geneus)
add_dependencies(task_assembly_geneus task_assembly_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_assembly_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg"
  "${MSG_I_FLAGS}"
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
)
_generate_msg_lisp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
)
_generate_msg_lisp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
)

### Generating Services
_generate_srv_lisp(task_assembly
  "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
)

### Generating Module File
_generate_module_lisp(task_assembly
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(task_assembly_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(task_assembly_generate_messages task_assembly_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_lisp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_lisp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_dependencies(task_assembly_generate_messages_lisp _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_lisp _task_assembly_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_assembly_genlisp)
add_dependencies(task_assembly_genlisp task_assembly_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_assembly_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg"
  "${MSG_I_FLAGS}"
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
)
_generate_msg_nodejs(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
)
_generate_msg_nodejs(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
)

### Generating Services
_generate_srv_nodejs(task_assembly
  "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
)

### Generating Module File
_generate_module_nodejs(task_assembly
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(task_assembly_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(task_assembly_generate_messages task_assembly_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_nodejs _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_nodejs _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_dependencies(task_assembly_generate_messages_nodejs _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_nodejs _task_assembly_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_assembly_gennodejs)
add_dependencies(task_assembly_gennodejs task_assembly_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_assembly_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg"
  "${MSG_I_FLAGS}"
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
)
_generate_msg_py(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
)
_generate_msg_py(task_assembly
  "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
)

### Generating Services
_generate_srv_py(task_assembly
  "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
)

### Generating Module File
_generate_module_py(task_assembly
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(task_assembly_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(task_assembly_generate_messages task_assembly_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBoxes3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_py _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/target.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_py _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/srv/door_open_planner.srv" NAME_WE)
add_dependencies(task_assembly_generate_messages_py _task_assembly_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/min/catkin_ws/src/task_assembly/msg/BoundingBox3d.msg" NAME_WE)
add_dependencies(task_assembly_generate_messages_py _task_assembly_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(task_assembly_genpy)
add_dependencies(task_assembly_genpy task_assembly_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS task_assembly_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/task_assembly
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(task_assembly_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(task_assembly_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(task_assembly_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(task_assembly_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/task_assembly
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(task_assembly_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(task_assembly_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(task_assembly_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(task_assembly_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/task_assembly
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(task_assembly_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(task_assembly_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(task_assembly_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(task_assembly_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/task_assembly
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(task_assembly_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(task_assembly_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(task_assembly_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(task_assembly_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/task_assembly
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(task_assembly_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(task_assembly_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(task_assembly_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(task_assembly_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
