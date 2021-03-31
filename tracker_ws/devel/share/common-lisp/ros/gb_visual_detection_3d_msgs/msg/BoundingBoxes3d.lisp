; Auto-generated. Do not edit!


(cl:in-package gb_visual_detection_3d_msgs-msg)


;//! \htmlinclude BoundingBoxes3d.msg.html

(cl:defclass <BoundingBoxes3d> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bounding_boxes
    :reader bounding_boxes
    :initarg :bounding_boxes
    :type (cl:vector gb_visual_detection_3d_msgs-msg:BoundingBox3d)
   :initform (cl:make-array 0 :element-type 'gb_visual_detection_3d_msgs-msg:BoundingBox3d :initial-element (cl:make-instance 'gb_visual_detection_3d_msgs-msg:BoundingBox3d))))
)

(cl:defclass BoundingBoxes3d (<BoundingBoxes3d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBoxes3d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBoxes3d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gb_visual_detection_3d_msgs-msg:<BoundingBoxes3d> is deprecated: use gb_visual_detection_3d_msgs-msg:BoundingBoxes3d instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BoundingBoxes3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gb_visual_detection_3d_msgs-msg:header-val is deprecated.  Use gb_visual_detection_3d_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bounding_boxes-val :lambda-list '(m))
(cl:defmethod bounding_boxes-val ((m <BoundingBoxes3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gb_visual_detection_3d_msgs-msg:bounding_boxes-val is deprecated.  Use gb_visual_detection_3d_msgs-msg:bounding_boxes instead.")
  (bounding_boxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBoxes3d>) ostream)
  "Serializes a message object of type '<BoundingBoxes3d>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bounding_boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bounding_boxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBoxes3d>) istream)
  "Deserializes a message object of type '<BoundingBoxes3d>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bounding_boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bounding_boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'gb_visual_detection_3d_msgs-msg:BoundingBox3d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBoxes3d>)))
  "Returns string type for a message object of type '<BoundingBoxes3d>"
  "gb_visual_detection_3d_msgs/BoundingBoxes3d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBoxes3d)))
  "Returns string type for a message object of type 'BoundingBoxes3d"
  "gb_visual_detection_3d_msgs/BoundingBoxes3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBoxes3d>)))
  "Returns md5sum for a message object of type '<BoundingBoxes3d>"
  "105285518cb176da52bda2a8cc5bd348")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBoxes3d)))
  "Returns md5sum for a message object of type 'BoundingBoxes3d"
  "105285518cb176da52bda2a8cc5bd348")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBoxes3d>)))
  "Returns full string definition for message of type '<BoundingBoxes3d>"
  (cl:format cl:nil "std_msgs/Header header~%BoundingBox3d[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gb_visual_detection_3d_msgs/BoundingBox3d~%string Class~%float64 probability~%float64 xmin~%float64 ymin~%float64 xmax~%float64 ymax~%float64 zmin~%float64 zmax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBoxes3d)))
  "Returns full string definition for message of type 'BoundingBoxes3d"
  (cl:format cl:nil "std_msgs/Header header~%BoundingBox3d[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gb_visual_detection_3d_msgs/BoundingBox3d~%string Class~%float64 probability~%float64 xmin~%float64 ymin~%float64 xmax~%float64 ymax~%float64 zmin~%float64 zmax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBoxes3d>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bounding_boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBoxes3d>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBoxes3d
    (cl:cons ':header (header msg))
    (cl:cons ':bounding_boxes (bounding_boxes msg))
))
