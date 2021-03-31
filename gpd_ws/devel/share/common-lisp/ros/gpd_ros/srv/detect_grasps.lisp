; Auto-generated. Do not edit!


(cl:in-package gpd_ros-srv)


;//! \htmlinclude detect_grasps-request.msg.html

(cl:defclass <detect_grasps-request> (roslisp-msg-protocol:ros-message)
  ((cloud_indexed
    :reader cloud_indexed
    :initarg :cloud_indexed
    :type gpd_ros-msg:CloudIndexed
    :initform (cl:make-instance 'gpd_ros-msg:CloudIndexed)))
)

(cl:defclass detect_grasps-request (<detect_grasps-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detect_grasps-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detect_grasps-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-srv:<detect_grasps-request> is deprecated: use gpd_ros-srv:detect_grasps-request instead.")))

(cl:ensure-generic-function 'cloud_indexed-val :lambda-list '(m))
(cl:defmethod cloud_indexed-val ((m <detect_grasps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-srv:cloud_indexed-val is deprecated.  Use gpd_ros-srv:cloud_indexed instead.")
  (cloud_indexed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detect_grasps-request>) ostream)
  "Serializes a message object of type '<detect_grasps-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_indexed) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detect_grasps-request>) istream)
  "Deserializes a message object of type '<detect_grasps-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_indexed) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detect_grasps-request>)))
  "Returns string type for a service object of type '<detect_grasps-request>"
  "gpd_ros/detect_graspsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detect_grasps-request)))
  "Returns string type for a service object of type 'detect_grasps-request"
  "gpd_ros/detect_graspsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detect_grasps-request>)))
  "Returns md5sum for a message object of type '<detect_grasps-request>"
  "6544a7e3669d79f6069fe9d81fee1c1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detect_grasps-request)))
  "Returns md5sum for a message object of type 'detect_grasps-request"
  "6544a7e3669d79f6069fe9d81fee1c1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detect_grasps-request>)))
  "Returns full string definition for message of type '<detect_grasps-request>"
  (cl:format cl:nil "gpd_ros/CloudIndexed cloud_indexed~%~%================================================================================~%MSG: gpd_ros/CloudIndexed~%# This message holds a point cloud and a list of indices into the point cloud ~%# at which to sample grasp candidates.~%~%# The point cloud.~%gpd_ros/CloudSources cloud_sources~%~%# The indices into the point cloud at which to sample grasp candidates.~%std_msgs/Int64[] indices~%~%================================================================================~%MSG: gpd_ros/CloudSources~%# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detect_grasps-request)))
  "Returns full string definition for message of type 'detect_grasps-request"
  (cl:format cl:nil "gpd_ros/CloudIndexed cloud_indexed~%~%================================================================================~%MSG: gpd_ros/CloudIndexed~%# This message holds a point cloud and a list of indices into the point cloud ~%# at which to sample grasp candidates.~%~%# The point cloud.~%gpd_ros/CloudSources cloud_sources~%~%# The indices into the point cloud at which to sample grasp candidates.~%std_msgs/Int64[] indices~%~%================================================================================~%MSG: gpd_ros/CloudSources~%# This message holds a point cloud that can be a combination of point clouds ~%# from different camera sources (at least one). For each point in the cloud, ~%# this message also stores the index of the camera that produced the point.~%~%# The point cloud.~%sensor_msgs/PointCloud2 cloud~%~%# For each point in the cloud, the index of the camera that acquired the point.~%std_msgs/Int64[] camera_source~%~%# A list of camera positions at which the point cloud was acquired.~%geometry_msgs/Point[] view_points~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detect_grasps-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_indexed))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detect_grasps-request>))
  "Converts a ROS message object to a list"
  (cl:list 'detect_grasps-request
    (cl:cons ':cloud_indexed (cloud_indexed msg))
))
;//! \htmlinclude detect_grasps-response.msg.html

(cl:defclass <detect_grasps-response> (roslisp-msg-protocol:ros-message)
  ((grasp_configs
    :reader grasp_configs
    :initarg :grasp_configs
    :type gpd_ros-msg:GraspConfigList
    :initform (cl:make-instance 'gpd_ros-msg:GraspConfigList)))
)

(cl:defclass detect_grasps-response (<detect_grasps-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detect_grasps-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detect_grasps-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-srv:<detect_grasps-response> is deprecated: use gpd_ros-srv:detect_grasps-response instead.")))

(cl:ensure-generic-function 'grasp_configs-val :lambda-list '(m))
(cl:defmethod grasp_configs-val ((m <detect_grasps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-srv:grasp_configs-val is deprecated.  Use gpd_ros-srv:grasp_configs instead.")
  (grasp_configs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detect_grasps-response>) ostream)
  "Serializes a message object of type '<detect_grasps-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp_configs) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detect_grasps-response>) istream)
  "Deserializes a message object of type '<detect_grasps-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp_configs) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detect_grasps-response>)))
  "Returns string type for a service object of type '<detect_grasps-response>"
  "gpd_ros/detect_graspsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detect_grasps-response)))
  "Returns string type for a service object of type 'detect_grasps-response"
  "gpd_ros/detect_graspsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detect_grasps-response>)))
  "Returns md5sum for a message object of type '<detect_grasps-response>"
  "6544a7e3669d79f6069fe9d81fee1c1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detect_grasps-response)))
  "Returns md5sum for a message object of type 'detect_grasps-response"
  "6544a7e3669d79f6069fe9d81fee1c1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detect_grasps-response>)))
  "Returns full string definition for message of type '<detect_grasps-response>"
  (cl:format cl:nil "gpd_ros/GraspConfigList grasp_configs~%~%~%================================================================================~%MSG: gpd_ros/GraspConfigList~%# This message stores a list of grasp configurations.~%~%# The time of acquisition, and the coordinate frame ID.~%Header header~%~%# The list of grasp configurations.~%gpd_ros/GraspConfig[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gpd_ros/GraspConfig~%# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detect_grasps-response)))
  "Returns full string definition for message of type 'detect_grasps-response"
  (cl:format cl:nil "gpd_ros/GraspConfigList grasp_configs~%~%~%================================================================================~%MSG: gpd_ros/GraspConfigList~%# This message stores a list of grasp configurations.~%~%# The time of acquisition, and the coordinate frame ID.~%Header header~%~%# The list of grasp configurations.~%gpd_ros/GraspConfig[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: gpd_ros/GraspConfig~%# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detect_grasps-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp_configs))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detect_grasps-response>))
  "Converts a ROS message object to a list"
  (cl:list 'detect_grasps-response
    (cl:cons ':grasp_configs (grasp_configs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'detect_grasps)))
  'detect_grasps-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'detect_grasps)))
  'detect_grasps-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detect_grasps)))
  "Returns string type for a service object of type '<detect_grasps>"
  "gpd_ros/detect_grasps")