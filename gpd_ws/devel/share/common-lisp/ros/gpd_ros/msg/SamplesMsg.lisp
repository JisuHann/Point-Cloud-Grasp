; Auto-generated. Do not edit!


(cl:in-package gpd_ros-msg)


;//! \htmlinclude SamplesMsg.msg.html

(cl:defclass <SamplesMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass SamplesMsg (<SamplesMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SamplesMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SamplesMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-msg:<SamplesMsg> is deprecated: use gpd_ros-msg:SamplesMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SamplesMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:header-val is deprecated.  Use gpd_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <SamplesMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:samples-val is deprecated.  Use gpd_ros-msg:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SamplesMsg>) ostream)
  "Serializes a message object of type '<SamplesMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SamplesMsg>) istream)
  "Deserializes a message object of type '<SamplesMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'samples)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SamplesMsg>)))
  "Returns string type for a message object of type '<SamplesMsg>"
  "gpd_ros/SamplesMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SamplesMsg)))
  "Returns string type for a message object of type 'SamplesMsg"
  "gpd_ros/SamplesMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SamplesMsg>)))
  "Returns md5sum for a message object of type '<SamplesMsg>"
  "26492e97ed8c13252c4a85592d3e93fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SamplesMsg)))
  "Returns md5sum for a message object of type 'SamplesMsg"
  "26492e97ed8c13252c4a85592d3e93fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SamplesMsg>)))
  "Returns full string definition for message of type '<SamplesMsg>"
  (cl:format cl:nil "# This message describes a set of point samples at which to detect grasps.~%~%# Header~%Header header~%~%# The samples, as (x,y,z) points, at which to search for grasp candidates. ~%geometry_msgs/Point[] samples~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SamplesMsg)))
  "Returns full string definition for message of type 'SamplesMsg"
  (cl:format cl:nil "# This message describes a set of point samples at which to detect grasps.~%~%# Header~%Header header~%~%# The samples, as (x,y,z) points, at which to search for grasp candidates. ~%geometry_msgs/Point[] samples~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SamplesMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SamplesMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SamplesMsg
    (cl:cons ':header (header msg))
    (cl:cons ':samples (samples msg))
))
