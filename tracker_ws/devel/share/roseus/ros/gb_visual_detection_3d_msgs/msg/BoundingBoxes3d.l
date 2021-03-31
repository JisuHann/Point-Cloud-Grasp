;; Auto-generated. Do not edit!


(when (boundp 'gb_visual_detection_3d_msgs::BoundingBoxes3d)
  (if (not (find-package "GB_VISUAL_DETECTION_3D_MSGS"))
    (make-package "GB_VISUAL_DETECTION_3D_MSGS"))
  (shadow 'BoundingBoxes3d (find-package "GB_VISUAL_DETECTION_3D_MSGS")))
(unless (find-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOXES3D")
  (make-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOXES3D"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes3d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gb_visual_detection_3d_msgs::BoundingBoxes3d
  :super ros::object
  :slots (_header _bounding_boxes ))

(defmethod gb_visual_detection_3d_msgs::BoundingBoxes3d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bounding_boxes __bounding_boxes) (let (r) (dotimes (i 0) (push (instance gb_visual_detection_3d_msgs::BoundingBox3d :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _bounding_boxes __bounding_boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance gb_visual_detection_3d_msgs::BoundingBox3d :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :md5sum-) "105285518cb176da52bda2a8cc5bd348")
(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :datatype-) "gb_visual_detection_3d_msgs/BoundingBoxes3d")
(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :definition-)
      "std_msgs/Header header
BoundingBox3d[] bounding_boxes

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
MSG: gb_visual_detection_3d_msgs/BoundingBox3d
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax

")



(provide :gb_visual_detection_3d_msgs/BoundingBoxes3d "105285518cb176da52bda2a8cc5bd348")


