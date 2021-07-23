; Auto-generated. Do not edit!


(cl:in-package task_assembly-srv)


;//! \htmlinclude door_open_planner-request.msg.html

(cl:defclass <door_open_planner-request> (roslisp-msg-protocol:ros-message)
  ((current_arm_joint_state
    :reader current_arm_joint_state
    :initarg :current_arm_joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (interpolate_path
    :reader interpolate_path
    :initarg :interpolate_path
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass door_open_planner-request (<door_open_planner-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <door_open_planner-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'door_open_planner-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_assembly-srv:<door_open_planner-request> is deprecated: use task_assembly-srv:door_open_planner-request instead.")))

(cl:ensure-generic-function 'current_arm_joint_state-val :lambda-list '(m))
(cl:defmethod current_arm_joint_state-val ((m <door_open_planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_assembly-srv:current_arm_joint_state-val is deprecated.  Use task_assembly-srv:current_arm_joint_state instead.")
  (current_arm_joint_state m))

(cl:ensure-generic-function 'interpolate_path-val :lambda-list '(m))
(cl:defmethod interpolate_path-val ((m <door_open_planner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_assembly-srv:interpolate_path-val is deprecated.  Use task_assembly-srv:interpolate_path instead.")
  (interpolate_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <door_open_planner-request>) ostream)
  "Serializes a message object of type '<door_open_planner-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_arm_joint_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'interpolate_path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <door_open_planner-request>) istream)
  "Deserializes a message object of type '<door_open_planner-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_arm_joint_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'interpolate_path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<door_open_planner-request>)))
  "Returns string type for a service object of type '<door_open_planner-request>"
  "task_assembly/door_open_plannerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'door_open_planner-request)))
  "Returns string type for a service object of type 'door_open_planner-request"
  "task_assembly/door_open_plannerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<door_open_planner-request>)))
  "Returns md5sum for a message object of type '<door_open_planner-request>"
  "b102701686e4ba528d2b5ce877e202a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'door_open_planner-request)))
  "Returns md5sum for a message object of type 'door_open_planner-request"
  "b102701686e4ba528d2b5ce877e202a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<door_open_planner-request>)))
  "Returns full string definition for message of type '<door_open_planner-request>"
  (cl:format cl:nil "sensor_msgs/JointState current_arm_joint_state~%std_msgs/Bool interpolate_path~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'door_open_planner-request)))
  "Returns full string definition for message of type 'door_open_planner-request"
  (cl:format cl:nil "sensor_msgs/JointState current_arm_joint_state~%std_msgs/Bool interpolate_path~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <door_open_planner-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_arm_joint_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'interpolate_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <door_open_planner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'door_open_planner-request
    (cl:cons ':current_arm_joint_state (current_arm_joint_state msg))
    (cl:cons ':interpolate_path (interpolate_path msg))
))
;//! \htmlinclude door_open_planner-response.msg.html

(cl:defclass <door_open_planner-response> (roslisp-msg-protocol:ros-message)
  ((joint_trajectory
    :reader joint_trajectory
    :initarg :joint_trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass door_open_planner-response (<door_open_planner-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <door_open_planner-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'door_open_planner-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_assembly-srv:<door_open_planner-response> is deprecated: use task_assembly-srv:door_open_planner-response instead.")))

(cl:ensure-generic-function 'joint_trajectory-val :lambda-list '(m))
(cl:defmethod joint_trajectory-val ((m <door_open_planner-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_assembly-srv:joint_trajectory-val is deprecated.  Use task_assembly-srv:joint_trajectory instead.")
  (joint_trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <door_open_planner-response>) ostream)
  "Serializes a message object of type '<door_open_planner-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <door_open_planner-response>) istream)
  "Deserializes a message object of type '<door_open_planner-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<door_open_planner-response>)))
  "Returns string type for a service object of type '<door_open_planner-response>"
  "task_assembly/door_open_plannerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'door_open_planner-response)))
  "Returns string type for a service object of type 'door_open_planner-response"
  "task_assembly/door_open_plannerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<door_open_planner-response>)))
  "Returns md5sum for a message object of type '<door_open_planner-response>"
  "b102701686e4ba528d2b5ce877e202a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'door_open_planner-response)))
  "Returns md5sum for a message object of type 'door_open_planner-response"
  "b102701686e4ba528d2b5ce877e202a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<door_open_planner-response>)))
  "Returns full string definition for message of type '<door_open_planner-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'door_open_planner-response)))
  "Returns full string definition for message of type 'door_open_planner-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <door_open_planner-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <door_open_planner-response>))
  "Converts a ROS message object to a list"
  (cl:list 'door_open_planner-response
    (cl:cons ':joint_trajectory (joint_trajectory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'door_open_planner)))
  'door_open_planner-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'door_open_planner)))
  'door_open_planner-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'door_open_planner)))
  "Returns string type for a service object of type '<door_open_planner>"
  "task_assembly/door_open_planner")