;; Auto-generated. Do not edit!


(when (boundp 'gpd_ros::detect_grasps)
  (if (not (find-package "GPD_ROS"))
    (make-package "GPD_ROS"))
  (shadow 'detect_grasps (find-package "GPD_ROS")))
(unless (find-package "GPD_ROS::DETECT_GRASPS")
  (make-package "GPD_ROS::DETECT_GRASPS"))
(unless (find-package "GPD_ROS::DETECT_GRASPSREQUEST")
  (make-package "GPD_ROS::DETECT_GRASPSREQUEST"))
(unless (find-package "GPD_ROS::DETECT_GRASPSRESPONSE")
  (make-package "GPD_ROS::DETECT_GRASPSRESPONSE"))

(in-package "ROS")





(defclass gpd_ros::detect_graspsRequest
  :super ros::object
  :slots (_cloud_indexed ))

(defmethod gpd_ros::detect_graspsRequest
  (:init
   (&key
    ((:cloud_indexed __cloud_indexed) (instance gpd_ros::CloudIndexed :init))
    )
   (send-super :init)
   (setq _cloud_indexed __cloud_indexed)
   self)
  (:cloud_indexed
   (&rest __cloud_indexed)
   (if (keywordp (car __cloud_indexed))
       (send* _cloud_indexed __cloud_indexed)
     (progn
       (if __cloud_indexed (setq _cloud_indexed (car __cloud_indexed)))
       _cloud_indexed)))
  (:serialization-length
   ()
   (+
    ;; gpd_ros/CloudIndexed _cloud_indexed
    (send _cloud_indexed :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; gpd_ros/CloudIndexed _cloud_indexed
       (send _cloud_indexed :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; gpd_ros/CloudIndexed _cloud_indexed
     (send _cloud_indexed :deserialize buf ptr-) (incf ptr- (send _cloud_indexed :serialization-length))
   ;;
   self)
  )

(defclass gpd_ros::detect_graspsResponse
  :super ros::object
  :slots (_grasp_configs ))

(defmethod gpd_ros::detect_graspsResponse
  (:init
   (&key
    ((:grasp_configs __grasp_configs) (instance gpd_ros::GraspConfigList :init))
    )
   (send-super :init)
   (setq _grasp_configs __grasp_configs)
   self)
  (:grasp_configs
   (&rest __grasp_configs)
   (if (keywordp (car __grasp_configs))
       (send* _grasp_configs __grasp_configs)
     (progn
       (if __grasp_configs (setq _grasp_configs (car __grasp_configs)))
       _grasp_configs)))
  (:serialization-length
   ()
   (+
    ;; gpd_ros/GraspConfigList _grasp_configs
    (send _grasp_configs :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; gpd_ros/GraspConfigList _grasp_configs
       (send _grasp_configs :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; gpd_ros/GraspConfigList _grasp_configs
     (send _grasp_configs :deserialize buf ptr-) (incf ptr- (send _grasp_configs :serialization-length))
   ;;
   self)
  )

(defclass gpd_ros::detect_grasps
  :super ros::object
  :slots ())

(setf (get gpd_ros::detect_grasps :md5sum-) "6544a7e3669d79f6069fe9d81fee1c1e")
(setf (get gpd_ros::detect_grasps :datatype-) "gpd_ros/detect_grasps")
(setf (get gpd_ros::detect_grasps :request) gpd_ros::detect_graspsRequest)
(setf (get gpd_ros::detect_grasps :response) gpd_ros::detect_graspsResponse)

(defmethod gpd_ros::detect_graspsRequest
  (:response () (instance gpd_ros::detect_graspsResponse :init)))

(setf (get gpd_ros::detect_graspsRequest :md5sum-) "6544a7e3669d79f6069fe9d81fee1c1e")
(setf (get gpd_ros::detect_graspsRequest :datatype-) "gpd_ros/detect_graspsRequest")
(setf (get gpd_ros::detect_graspsRequest :definition-)
      "gpd_ros/CloudIndexed cloud_indexed

================================================================================
MSG: gpd_ros/CloudIndexed
# This message holds a point cloud and a list of indices into the point cloud 
# at which to sample grasp candidates.

# The point cloud.
gpd_ros/CloudSources cloud_sources

# The indices into the point cloud at which to sample grasp candidates.
std_msgs/Int64[] indices

================================================================================
MSG: gpd_ros/CloudSources
# This message holds a point cloud that can be a combination of point clouds 
# from different camera sources (at least one). For each point in the cloud, 
# this message also stores the index of the camera that produced the point.

# The point cloud.
sensor_msgs/PointCloud2 cloud

# For each point in the cloud, the index of the camera that acquired the point.
std_msgs/Int64[] camera_source

# A list of camera positions at which the point cloud was acquired.
geometry_msgs/Point[] view_points
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
gpd_ros/GraspConfigList grasp_configs


================================================================================
MSG: gpd_ros/GraspConfigList
# This message stores a list of grasp configurations.

# The time of acquisition, and the coordinate frame ID.
Header header

# The list of grasp configurations.
gpd_ros/GraspConfig[] grasps

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
MSG: gpd_ros/GraspConfig
# This message describes a 2-finger grasp configuration by its 6-DOF pose,
# consisting of a 3-DOF position and 3-DOF orientation, and the opening
# width of the robot hand.

# Position
geometry_msgs/Point position # grasp position (bottom/base center of robot hand)

# Orientation represented as three axis (R = [approach binormal axis])
geometry_msgs/Vector3 approach # grasp approach direction
geometry_msgs/Vector3 binormal # hand closing direction
geometry_msgs/Vector3 axis # hand axis

std_msgs/Float32 width # Required aperture (opening width) of the robot hand

std_msgs/Float32 score # Score assigned to the grasp by the classifier

geometry_msgs/Point sample # point at which the grasp was found

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float32
")

(setf (get gpd_ros::detect_graspsResponse :md5sum-) "6544a7e3669d79f6069fe9d81fee1c1e")
(setf (get gpd_ros::detect_graspsResponse :datatype-) "gpd_ros/detect_graspsResponse")
(setf (get gpd_ros::detect_graspsResponse :definition-)
      "gpd_ros/CloudIndexed cloud_indexed

================================================================================
MSG: gpd_ros/CloudIndexed
# This message holds a point cloud and a list of indices into the point cloud 
# at which to sample grasp candidates.

# The point cloud.
gpd_ros/CloudSources cloud_sources

# The indices into the point cloud at which to sample grasp candidates.
std_msgs/Int64[] indices

================================================================================
MSG: gpd_ros/CloudSources
# This message holds a point cloud that can be a combination of point clouds 
# from different camera sources (at least one). For each point in the cloud, 
# this message also stores the index of the camera that produced the point.

# The point cloud.
sensor_msgs/PointCloud2 cloud

# For each point in the cloud, the index of the camera that acquired the point.
std_msgs/Int64[] camera_source

# A list of camera positions at which the point cloud was acquired.
geometry_msgs/Point[] view_points
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
gpd_ros/GraspConfigList grasp_configs


================================================================================
MSG: gpd_ros/GraspConfigList
# This message stores a list of grasp configurations.

# The time of acquisition, and the coordinate frame ID.
Header header

# The list of grasp configurations.
gpd_ros/GraspConfig[] grasps

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
MSG: gpd_ros/GraspConfig
# This message describes a 2-finger grasp configuration by its 6-DOF pose,
# consisting of a 3-DOF position and 3-DOF orientation, and the opening
# width of the robot hand.

# Position
geometry_msgs/Point position # grasp position (bottom/base center of robot hand)

# Orientation represented as three axis (R = [approach binormal axis])
geometry_msgs/Vector3 approach # grasp approach direction
geometry_msgs/Vector3 binormal # hand closing direction
geometry_msgs/Vector3 axis # hand axis

std_msgs/Float32 width # Required aperture (opening width) of the robot hand

std_msgs/Float32 score # Score assigned to the grasp by the classifier

geometry_msgs/Point sample # point at which the grasp was found

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float32
")



(provide :gpd_ros/detect_grasps "6544a7e3669d79f6069fe9d81fee1c1e")


