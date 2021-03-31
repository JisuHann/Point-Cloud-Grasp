; Auto-generated. Do not edit!


(cl:in-package gpd_ros-msg)


;//! \htmlinclude GraspConfigList.msg.html

(cl:defclass <GraspConfigList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (grasps
    :reader grasps
    :initarg :grasps
    :type (cl:vector gpd_ros-msg:GraspConfig)
   :initform (cl:make-array 0 :element-type 'gpd_ros-msg:GraspConfig :initial-element (cl:make-instance 'gpd_ros-msg:GraspConfig))))
)

(cl:defclass GraspConfigList (<GraspConfigList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspConfigList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspConfigList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-msg:<GraspConfigList> is deprecated: use gpd_ros-msg:GraspConfigList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GraspConfigList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:header-val is deprecated.  Use gpd_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'grasps-val :lambda-list '(m))
(cl:defmethod grasps-val ((m <GraspConfigList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:grasps-val is deprecated.  Use gpd_ros-msg:grasps instead.")
  (grasps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspConfigList>) ostream)
  "Serializes a message object of type '<GraspConfigList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspConfigList>) istream)
  "Deserializes a message object of type '<GraspConfigList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'gpd_ros-msg:GraspConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspConfigList>)))
  "Returns string type for a message object of type '<GraspConfigList>"
  "gpd_ros/GraspConfigList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspConfigList)))
  "Returns string type for a message object of type 'GraspConfigList"
  "gpd_ros/GraspConfigList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspConfigList>)))
  "Returns md5sum for a message object of type '<GraspConfigList>"
  "b023d624ebdc1464392309d191c14641")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspConfigList)))
  "Returns md5sum for a message object of type 'GraspConfigList"
  "b023d624ebdc1464392309d191c14641")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspConfigList>)))
  "Returns full string definition for message of type '<GraspConfigList>"
  (cl:format cl:nil "# This message stores a list of grasp configurations.~%~%# The time of acquisition, and the coordinate frame ID.~%Header header~%~%# The list of grasp configurations.~%gpd_ros/GraspConfig[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gpd_ros/GraspConfig~%# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspConfigList)))
  "Returns full string definition for message of type 'GraspConfigList"
  (cl:format cl:nil "# This message stores a list of grasp configurations.~%~%# The time of acquisition, and the coordinate frame ID.~%Header header~%~%# The list of grasp configurations.~%gpd_ros/GraspConfig[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gpd_ros/GraspConfig~%# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspConfigList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspConfigList>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspConfigList
    (cl:cons ':header (header msg))
    (cl:cons ':grasps (grasps msg))
))
