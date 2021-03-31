;; Auto-generated. Do not edit!


(when (boundp 'gpd_ros::CloudSamples)
  (if (not (find-package "GPD_ROS"))
    (make-package "GPD_ROS"))
  (shadow 'CloudSamples (find-package "GPD_ROS")))
(unless (find-package "GPD_ROS::CLOUDSAMPLES")
  (make-package "GPD_ROS::CLOUDSAMPLES"))

(in-package "ROS")
;;//! \htmlinclude CloudSamples.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass gpd_ros::CloudSamples
  :super ros::object
  :slots (_cloud_sources _samples ))

(defmethod gpd_ros::CloudSamples
  (:init
   (&key
    ((:cloud_sources __cloud_sources) (instance gpd_ros::CloudSources :init))
    ((:samples __samples) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _cloud_sources __cloud_sources)
   (setq _samples __samples)
   self)
  (:cloud_sources
   (&rest __cloud_sources)
   (if (keywordp (car __cloud_sources))
       (send* _cloud_sources __cloud_sources)
     (progn
       (if __cloud_sources (setq _cloud_sources (car __cloud_sources)))
       _cloud_sources)))
  (:samples
   (&rest __samples)
   (if (keywordp (car __samples))
       (send* _samples __samples)
     (progn
       (if __samples (setq _samples (car __samples)))
       _samples)))
  (:serialization-length
   ()
   (+
    ;; gpd_ros/CloudSources _cloud_sources
    (send _cloud_sources :serialization-length)
    ;; geometry_msgs/Point[] _samples
    (apply #'+ (send-all _samples :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; gpd_ros/CloudSources _cloud_sources
       (send _cloud_sources :serialize s)
     ;; geometry_msgs/Point[] _samples
     (write-long (length _samples) s)
     (dolist (elem _samples)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; gpd_ros/CloudSources _cloud_sources
     (send _cloud_sources :deserialize buf ptr-) (incf ptr- (send _cloud_sources :serialization-length))
   ;; geometry_msgs/Point[] _samples
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _samples (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _samples)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get gpd_ros::CloudSamples :md5sum-) "7acb8b5070bbe8bcd578eb420ebc4d9b")
(setf (get gpd_ros::CloudSamples :datatype-) "gpd_ros/CloudSamples")
(setf (get gpd_ros::CloudSamples :definition-)
      "# This message holds a point cloud and a list of samples at which the grasp 
# detector should search for grasp candidates.

# The point cloud.
gpd_ros/CloudSources cloud_sources

# The samples, as (x,y,z) points, at which to search for grasp candidates. 
geometry_msgs/Point[] samples

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

")



(provide :gpd_ros/CloudSamples "7acb8b5070bbe8bcd578eb420ebc4d9b")


