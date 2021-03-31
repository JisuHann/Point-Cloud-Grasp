; Auto-generated. Do not edit!


(cl:in-package gpd_ros-msg)


;//! \htmlinclude CloudSources.msg.html

(cl:defclass <CloudSources> (roslisp-msg-protocol:ros-message)
  ((cloud
    :reader cloud
    :initarg :cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (camera_source
    :reader camera_source
    :initarg :camera_source
    :type (cl:vector std_msgs-msg:Int64)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Int64 :initial-element (cl:make-instance 'std_msgs-msg:Int64)))
   (view_points
    :reader view_points
    :initarg :view_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass CloudSources (<CloudSources>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloudSources>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloudSources)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-msg:<CloudSources> is deprecated: use gpd_ros-msg:CloudSources instead.")))

(cl:ensure-generic-function 'cloud-val :lambda-list '(m))
(cl:defmethod cloud-val ((m <CloudSources>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:cloud-val is deprecated.  Use gpd_ros-msg:cloud instead.")
  (cloud m))

(cl:ensure-generic-function 'camera_source-val :lambda-list '(m))
(cl:defmethod camera_source-val ((m <CloudSources>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:camera_source-val is deprecated.  Use gpd_ros-msg:camera_source instead.")
  (camera_source m))

(cl:ensure-generic-function 'view_points-val :lambda-list '(m))
(cl:defmethod view_points-val ((m <CloudSources>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:view_points-val is deprecated.  Use gpd_ros-msg:view_points instead.")
  (view_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloudSources>) ostream)
  "Serializes a message object of type '<CloudSources>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'camera_source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'camera_source))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'view_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'view_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloudSources>) istream)
  "Deserializes a message object of type '<CloudSources>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'camera_source) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'camera_source)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Int64))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'view_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'view_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloudSources>)))
  "Returns string type for a message object of type '<CloudSources>"
  "gpd_ros/CloudSources")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloudSources)))
  "Returns string type for a message object of type 'CloudSources"
  "gpd_ros/CloudSources")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloudSources>)))
  "Returns md5sum for a message object of type '<CloudSources>"
  "db42f0bd3c98d8b681c7942579de88bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloudSources)))
  "Returns md5sum for a message object of type 'CloudSources"
  "db42f0bd3c98d8b681c7942579de88bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloudSources>)))
  "Returns full string definition for message of type '<CloudSources>"
  (cl:format cl:nil "# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloudSources)))
  "Returns full string definition for message of type 'CloudSources"
  (cl:format cl:nil "# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloudSources>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'camera_source) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'view_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloudSources>))
  "Converts a ROS message object to a list"
  (cl:list 'CloudSources
    (cl:cons ':cloud (cloud msg))
    (cl:cons ':camera_source (camera_source msg))
    (cl:cons ':view_points (view_points msg))
))
