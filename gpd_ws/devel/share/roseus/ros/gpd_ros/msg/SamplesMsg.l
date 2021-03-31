;; Auto-generated. Do not edit!


(when (boundp 'gpd_ros::SamplesMsg)
  (if (not (find-package "GPD_ROS"))
    (make-package "GPD_ROS"))
  (shadow 'SamplesMsg (find-package "GPD_ROS")))
(unless (find-package "GPD_ROS::SAMPLESMSG")
  (make-package "GPD_ROS::SAMPLESMSG"))

(in-package "ROS")
;;//! \htmlinclude SamplesMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gpd_ros::SamplesMsg
  :super ros::object
  :slots (_header _samples ))

(defmethod gpd_ros::SamplesMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:samples __samples) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _samples __samples)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _samples
    (apply #'+ (send-all _samples :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _samples
     (write-long (length _samples) s)
     (dolist (elem _samples)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
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

(setf (get gpd_ros::SamplesMsg :md5sum-) "26492e97ed8c13252c4a85592d3e93fd")
(setf (get gpd_ros::SamplesMsg :datatype-) "gpd_ros/SamplesMsg")
(setf (get gpd_ros::SamplesMsg :definition-)
      "# This message describes a set of point samples at which to detect grasps.

# Header
Header header

# The samples, as (x,y,z) points, at which to search for grasp candidates. 
geometry_msgs/Point[] samples

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :gpd_ros/SamplesMsg "26492e97ed8c13252c4a85592d3e93fd")


