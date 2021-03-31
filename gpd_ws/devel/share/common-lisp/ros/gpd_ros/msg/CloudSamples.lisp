; Auto-generated. Do not edit!


(cl:in-package gpd_ros-msg)


;//! \htmlinclude CloudSamples.msg.html

(cl:defclass <CloudSamples> (roslisp-msg-protocol:ros-message)
  ((cloud_sources
    :reader cloud_sources
    :initarg :cloud_sources
    :type gpd_ros-msg:CloudSources
    :initform (cl:make-instance 'gpd_ros-msg:CloudSources))
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass CloudSamples (<CloudSamples>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloudSamples>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloudSamples)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-msg:<CloudSamples> is deprecated: use gpd_ros-msg:CloudSamples instead.")))

(cl:ensure-generic-function 'cloud_sources-val :lambda-list '(m))
(cl:defmethod cloud_sources-val ((m <CloudSamples>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:cloud_sources-val is deprecated.  Use gpd_ros-msg:cloud_sources instead.")
  (cloud_sources m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <CloudSamples>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:samples-val is deprecated.  Use gpd_ros-msg:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloudSamples>) ostream)
  "Serializes a message object of type '<CloudSamples>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_sources) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloudSamples>) istream)
  "Deserializes a message object of type '<CloudSamples>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_sources) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloudSamples>)))
  "Returns string type for a message object of type '<CloudSamples>"
  "gpd_ros/CloudSamples")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloudSamples)))
  "Returns string type for a message object of type 'CloudSamples"
  "gpd_ros/CloudSamples")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloudSamples>)))
  "Returns md5sum for a message object of type '<CloudSamples>"
  "7acb8b5070bbe8bcd578eb420ebc4d9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloudSamples)))
  "Returns md5sum for a message object of type 'CloudSamples"
  "7acb8b5070bbe8bcd578eb420ebc4d9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloudSamples>)))
  "Returns full string definition for message of type '<CloudSamples>"
  (cl:format cl:nil "# This message holds a point cloud and a list of samples at which the grasp ~%# detector should search for grasp candidates.~%~%# The point cloud.~%gpd_ros/CloudSources cloud_sources~%~%# The samples, as (x,y,z) points, at which to search for grasp candidates. ~%geometry_msgs/Point[] samples~%~%================================================================================~%MSG: gpd_ros/CloudSources~%# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloudSamples)))
  "Returns full string definition for message of type 'CloudSamples"
  (cl:format cl:nil "# This message holds a point cloud and a list of samples at which the grasp ~%# detector should search for grasp candidates.~%~%# The point cloud.~%gpd_ros/CloudSources cloud_sources~%~%# The samples, as (x,y,z) points, at which to search for grasp candidates. ~%geometry_msgs/Point[] samples~%~%================================================================================~%MSG: gpd_ros/CloudSources~%# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloudSamples>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_sources))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloudSamples>))
  "Converts a ROS message object to a list"
  (cl:list 'CloudSamples
    (cl:cons ':cloud_sources (cloud_sources msg))
    (cl:cons ':samples (samples msg))
))
