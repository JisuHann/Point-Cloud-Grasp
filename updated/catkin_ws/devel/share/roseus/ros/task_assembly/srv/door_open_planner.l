;; Auto-generated. Do not edit!


(when (boundp 'task_assembly::door_open_planner)
  (if (not (find-package "TASK_ASSEMBLY"))
    (make-package "TASK_ASSEMBLY"))
  (shadow 'door_open_planner (find-package "TASK_ASSEMBLY")))
(unless (find-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNER")
  (make-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNER"))
(unless (find-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNERREQUEST")
  (make-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNERREQUEST"))
(unless (find-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNERRESPONSE")
  (make-package "TASK_ASSEMBLY::DOOR_OPEN_PLANNERRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass task_assembly::door_open_plannerRequest
  :super ros::object
  :slots (_current_arm_joint_state _interpolate_path ))

(defmethod task_assembly::door_open_plannerRequest
  (:init
   (&key
    ((:current_arm_joint_state __current_arm_joint_state) (instance sensor_msgs::JointState :init))
    ((:interpolate_path __interpolate_path) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _current_arm_joint_state __current_arm_joint_state)
   (setq _interpolate_path __interpolate_path)
   self)
  (:current_arm_joint_state
   (&rest __current_arm_joint_state)
   (if (keywordp (car __current_arm_joint_state))
       (send* _current_arm_joint_state __current_arm_joint_state)
     (progn
       (if __current_arm_joint_state (setq _current_arm_joint_state (car __current_arm_joint_state)))
       _current_arm_joint_state)))
  (:interpolate_path
   (&rest __interpolate_path)
   (if (keywordp (car __interpolate_path))
       (send* _interpolate_path __interpolate_path)
     (progn
       (if __interpolate_path (setq _interpolate_path (car __interpolate_path)))
       _interpolate_path)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _current_arm_joint_state
    (send _current_arm_joint_state :serialization-length)
    ;; std_msgs/Bool _interpolate_path
    (send _interpolate_path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _current_arm_joint_state
       (send _current_arm_joint_state :serialize s)
     ;; std_msgs/Bool _interpolate_path
       (send _interpolate_path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _current_arm_joint_state
     (send _current_arm_joint_state :deserialize buf ptr-) (incf ptr- (send _current_arm_joint_state :serialization-length))
   ;; std_msgs/Bool _interpolate_path
     (send _interpolate_path :deserialize buf ptr-) (incf ptr- (send _interpolate_path :serialization-length))
   ;;
   self)
  )

(defclass task_assembly::door_open_plannerResponse
  :super ros::object
  :slots (_joint_trajectory ))

(defmethod task_assembly::door_open_plannerResponse
  (:init
   (&key
    ((:joint_trajectory __joint_trajectory) (instance trajectory_msgs::JointTrajectory :init))
    )
   (send-super :init)
   (setq _joint_trajectory __joint_trajectory)
   self)
  (:joint_trajectory
   (&rest __joint_trajectory)
   (if (keywordp (car __joint_trajectory))
       (send* _joint_trajectory __joint_trajectory)
     (progn
       (if __joint_trajectory (setq _joint_trajectory (car __joint_trajectory)))
       _joint_trajectory)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _joint_trajectory
    (send _joint_trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _joint_trajectory
       (send _joint_trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _joint_trajectory
     (send _joint_trajectory :deserialize buf ptr-) (incf ptr- (send _joint_trajectory :serialization-length))
   ;;
   self)
  )

(defclass task_assembly::door_open_planner
  :super ros::object
  :slots ())

(setf (get task_assembly::door_open_planner :md5sum-) "b102701686e4ba528d2b5ce877e202a8")
(setf (get task_assembly::door_open_planner :datatype-) "task_assembly/door_open_planner")
(setf (get task_assembly::door_open_planner :request) task_assembly::door_open_plannerRequest)
(setf (get task_assembly::door_open_planner :response) task_assembly::door_open_plannerResponse)

(defmethod task_assembly::door_open_plannerRequest
  (:response () (instance task_assembly::door_open_plannerResponse :init)))

(setf (get task_assembly::door_open_plannerRequest :md5sum-) "b102701686e4ba528d2b5ce877e202a8")
(setf (get task_assembly::door_open_plannerRequest :datatype-) "task_assembly/door_open_plannerRequest")
(setf (get task_assembly::door_open_plannerRequest :definition-)
      "sensor_msgs/JointState current_arm_joint_state
std_msgs/Bool interpolate_path

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/Bool
---
trajectory_msgs/JointTrajectory joint_trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")

(setf (get task_assembly::door_open_plannerResponse :md5sum-) "b102701686e4ba528d2b5ce877e202a8")
(setf (get task_assembly::door_open_plannerResponse :datatype-) "task_assembly/door_open_plannerResponse")
(setf (get task_assembly::door_open_plannerResponse :definition-)
      "sensor_msgs/JointState current_arm_joint_state
std_msgs/Bool interpolate_path

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/Bool
---
trajectory_msgs/JointTrajectory joint_trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")



(provide :task_assembly/door_open_planner "b102701686e4ba528d2b5ce877e202a8")


