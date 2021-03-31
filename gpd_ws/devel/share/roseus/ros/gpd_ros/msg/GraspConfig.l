;; Auto-generated. Do not edit!


(when (boundp 'gpd_ros::GraspConfig)
  (if (not (find-package "GPD_ROS"))
    (make-package "GPD_ROS"))
  (shadow 'GraspConfig (find-package "GPD_ROS")))
(unless (find-package "GPD_ROS::GRASPCONFIG")
  (make-package "GPD_ROS::GRASPCONFIG"))

(in-package "ROS")
;;//! \htmlinclude GraspConfig.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gpd_ros::GraspConfig
  :super ros::object
  :slots (_position _approach _binormal _axis _width _score _sample ))

(defmethod gpd_ros::GraspConfig
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:approach __approach) (instance geometry_msgs::Vector3 :init))
    ((:binormal __binormal) (instance geometry_msgs::Vector3 :init))
    ((:axis __axis) (instance geometry_msgs::Vector3 :init))
    ((:width __width) (instance std_msgs::Float32 :init))
    ((:score __score) (instance std_msgs::Float32 :init))
    ((:sample __sample) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _approach __approach)
   (setq _binormal __binormal)
   (setq _axis __axis)
   (setq _width __width)
   (setq _score __score)
   (setq _sample __sample)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:approach
   (&rest __approach)
   (if (keywordp (car __approach))
       (send* _approach __approach)
     (progn
       (if __approach (setq _approach (car __approach)))
       _approach)))
  (:binormal
   (&rest __binormal)
   (if (keywordp (car __binormal))
       (send* _binormal __binormal)
     (progn
       (if __binormal (setq _binormal (car __binormal)))
       _binormal)))
  (:axis
   (&rest __axis)
   (if (keywordp (car __axis))
       (send* _axis __axis)
     (progn
       (if __axis (setq _axis (car __axis)))
       _axis)))
  (:width
   (&rest __width)
   (if (keywordp (car __width))
       (send* _width __width)
     (progn
       (if __width (setq _width (car __width)))
       _width)))
  (:score
   (&rest __score)
   (if (keywordp (car __score))
       (send* _score __score)
     (progn
       (if __score (setq _score (car __score)))
       _score)))
  (:sample
   (&rest __sample)
   (if (keywordp (car __sample))
       (send* _sample __sample)
     (progn
       (if __sample (setq _sample (car __sample)))
       _sample)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _approach
    (send _approach :serialization-length)
    ;; geometry_msgs/Vector3 _binormal
    (send _binormal :serialization-length)
    ;; geometry_msgs/Vector3 _axis
    (send _axis :serialization-length)
    ;; std_msgs/Float32 _width
    (send _width :serialization-length)
    ;; std_msgs/Float32 _score
    (send _score :serialization-length)
    ;; geometry_msgs/Point _sample
    (send _sample :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _approach
       (send _approach :serialize s)
     ;; geometry_msgs/Vector3 _binormal
       (send _binormal :serialize s)
     ;; geometry_msgs/Vector3 _axis
       (send _axis :serialize s)
     ;; std_msgs/Float32 _width
       (send _width :serialize s)
     ;; std_msgs/Float32 _score
       (send _score :serialize s)
     ;; geometry_msgs/Point _sample
       (send _sample :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _approach
     (send _approach :deserialize buf ptr-) (incf ptr- (send _approach :serialization-length))
   ;; geometry_msgs/Vector3 _binormal
     (send _binormal :deserialize buf ptr-) (incf ptr- (send _binormal :serialization-length))
   ;; geometry_msgs/Vector3 _axis
     (send _axis :deserialize buf ptr-) (incf ptr- (send _axis :serialization-length))
   ;; std_msgs/Float32 _width
     (send _width :deserialize buf ptr-) (incf ptr- (send _width :serialization-length))
   ;; std_msgs/Float32 _score
     (send _score :deserialize buf ptr-) (incf ptr- (send _score :serialization-length))
   ;; geometry_msgs/Point _sample
     (send _sample :deserialize buf ptr-) (incf ptr- (send _sample :serialization-length))
   ;;
   self)
  )

(setf (get gpd_ros::GraspConfig :md5sum-) "8753a773793263ef11dce97fd6d996d5")
(setf (get gpd_ros::GraspConfig :datatype-) "gpd_ros/GraspConfig")
(setf (get gpd_ros::GraspConfig :definition-)
      "# This message describes a 2-finger grasp configuration by its 6-DOF pose,
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
float32 data
")



(provide :gpd_ros/GraspConfig "8753a773793263ef11dce97fd6d996d5")


