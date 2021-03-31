; Auto-generated. Do not edit!


(cl:in-package gpd_ros-msg)


;//! \htmlinclude GraspConfig.msg.html

(cl:defclass <GraspConfig> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (approach
    :reader approach
    :initarg :approach
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (binormal
    :reader binormal
    :initarg :binormal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (axis
    :reader axis
    :initarg :axis
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (width
    :reader width
    :initarg :width
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (score
    :reader score
    :initarg :score
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (sample
    :reader sample
    :initarg :sample
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass GraspConfig (<GraspConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpd_ros-msg:<GraspConfig> is deprecated: use gpd_ros-msg:GraspConfig instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:position-val is deprecated.  Use gpd_ros-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'approach-val :lambda-list '(m))
(cl:defmethod approach-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:approach-val is deprecated.  Use gpd_ros-msg:approach instead.")
  (approach m))

(cl:ensure-generic-function 'binormal-val :lambda-list '(m))
(cl:defmethod binormal-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:binormal-val is deprecated.  Use gpd_ros-msg:binormal instead.")
  (binormal m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:axis-val is deprecated.  Use gpd_ros-msg:axis instead.")
  (axis m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:width-val is deprecated.  Use gpd_ros-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:score-val is deprecated.  Use gpd_ros-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'sample-val :lambda-list '(m))
(cl:defmethod sample-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpd_ros-msg:sample-val is deprecated.  Use gpd_ros-msg:sample instead.")
  (sample m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspConfig>) ostream)
  "Serializes a message object of type '<GraspConfig>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'approach) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'binormal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'axis) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'width) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'score) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sample) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspConfig>) istream)
  "Deserializes a message object of type '<GraspConfig>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'approach) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'binormal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'axis) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'width) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'score) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sample) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspConfig>)))
  "Returns string type for a message object of type '<GraspConfig>"
  "gpd_ros/GraspConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspConfig)))
  "Returns string type for a message object of type 'GraspConfig"
  "gpd_ros/GraspConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspConfig>)))
  "Returns md5sum for a message object of type '<GraspConfig>"
  "8753a773793263ef11dce97fd6d996d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspConfig)))
  "Returns md5sum for a message object of type 'GraspConfig"
  "8753a773793263ef11dce97fd6d996d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspConfig>)))
  "Returns full string definition for message of type '<GraspConfig>"
  (cl:format cl:nil "# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspConfig)))
  "Returns full string definition for message of type 'GraspConfig"
  (cl:format cl:nil "# This message describes a 2-finger grasp configuration by its 6-DOF pose,~%# consisting of a 3-DOF position and 3-DOF orientation, and the opening~%# width of the robot hand.~%~%# Position~%geometry_msgs/Point position # grasp position (bottom/base center of robot hand)~%~%# Orientation represented as three axis (R = [approach binormal axis])~%geometry_msgs/Vector3 approach # grasp approach direction~%geometry_msgs/Vector3 binormal # hand closing direction~%geometry_msgs/Vector3 axis # hand axis~%~%std_msgs/Float32 width # Required aperture (opening width) of the robot hand~%~%std_msgs/Float32 score # Score assigned to the grasp by the classifier~%~%geometry_msgs/Point sample # point at which the grasp was found~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspConfig>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'approach))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'binormal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'axis))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'width))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'score))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sample))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspConfig
    (cl:cons ':position (position msg))
    (cl:cons ':approach (approach msg))
    (cl:cons ':binormal (binormal msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':width (width msg))
    (cl:cons ':score (score msg))
    (cl:cons ':sample (sample msg))
))
