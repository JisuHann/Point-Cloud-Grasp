;; Auto-generated. Do not edit!


(when (boundp 'gpd_ros::CloudSources)
  (if (not (find-package "GPD_ROS"))
    (make-package "GPD_ROS"))
  (shadow 'CloudSources (find-package "GPD_ROS")))
(unless (find-package "GPD_ROS::CLOUDSOURCES")
  (make-package "GPD_ROS::CLOUDSOURCES"))

(in-package "ROS")
;;//! \htmlinclude CloudSources.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gpd_ros::CloudSources
  :super ros::object
  :slots (_cloud _camera_source _view_points ))

(defmethod gpd_ros::CloudSources
  (:init
   (&key
    ((:cloud __cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:camera_source __camera_source) (let (r) (dotimes (i 0) (push (instance std_msgs::Int64 :init) r)) r))
    ((:view_points __view_points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _cloud __cloud)
   (setq _camera_source __camera_source)
   (setq _view_points __view_points)
   self)
  (:cloud
   (&rest __cloud)
   (if (keywordp (car __cloud))
       (send* _cloud __cloud)
     (progn
       (if __cloud (setq _cloud (car __cloud)))
       _cloud)))
  (:camera_source
   (&rest __camera_source)
   (if (keywordp (car __camera_source))
       (send* _camera_source __camera_source)
     (progn
       (if __camera_source (setq _camera_source (car __camera_source)))
       _camera_source)))
  (:view_points
   (&rest __view_points)
   (if (keywordp (car __view_points))
       (send* _view_points __view_points)
     (progn
       (if __view_points (setq _view_points (car __view_points)))
       _view_points)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _cloud
    (send _cloud :serialization-length)
    ;; std_msgs/Int64[] _camera_source
    (apply #'+ (send-all _camera_source :serialization-length)) 4
    ;; geometry_msgs/Point[] _view_points
    (apply #'+ (send-all _view_points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _cloud
       (send _cloud :serialize s)
     ;; std_msgs/Int64[] _camera_source
     (write-long (length _camera_source) s)
     (dolist (elem _camera_source)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _view_points
     (write-long (length _view_points) s)
     (dolist (elem _view_points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _cloud
     (send _cloud :deserialize buf ptr-) (incf ptr- (send _cloud :serialization-length))
   ;; std_msgs/Int64[] _camera_source
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _camera_source (let (r) (dotimes (i n) (push (instance std_msgs::Int64 :init) r)) r))
     (dolist (elem- _camera_source)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _view_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _view_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _view_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get gpd_ros::CloudSources :md5sum-) "db42f0bd3c98d8b681c7942579de88bd")
(setf (get gpd_ros::CloudSources :datatype-) "gpd_ros/CloudSources")
(setf (get gpd_ros::CloudSources :definition-)
      "# This message holds a point cloud that can be a combination of point clouds 
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

")



(provide :gpd_ros/CloudSources "db42f0bd3c98d8b681c7942579de88bd")


