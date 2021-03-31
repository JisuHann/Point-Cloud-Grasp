;; Auto-generated. Do not edit!


(when (boundp 'gb_visual_detection_3d_msgs::BoundingBox3d)
  (if (not (find-package "GB_VISUAL_DETECTION_3D_MSGS"))
    (make-package "GB_VISUAL_DETECTION_3D_MSGS"))
  (shadow 'BoundingBox3d (find-package "GB_VISUAL_DETECTION_3D_MSGS")))
(unless (find-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOX3D")
  (make-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOX3D"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox3d.msg.html


(defclass gb_visual_detection_3d_msgs::BoundingBox3d
  :super ros::object
  :slots (_Class _probability _xmin _ymin _xmax _ymax _zmin _zmax ))

(defmethod gb_visual_detection_3d_msgs::BoundingBox3d
  (:init
   (&key
    ((:Class __Class) "")
    ((:probability __probability) 0.0)
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    ((:zmin __zmin) 0.0)
    ((:zmax __zmax) 0.0)
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _probability (float __probability))
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   (setq _zmin (float __zmin))
   (setq _zmax (float __zmax))
   self)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:zmin
   (&optional __zmin)
   (if __zmin (setq _zmin __zmin)) _zmin)
  (:zmax
   (&optional __zmax)
   (if __zmax (setq _zmax __zmax)) _zmax)
  (:serialization-length
   ()
   (+
    ;; string _Class
    4 (length _Class)
    ;; float64 _probability
    8
    ;; float64 _xmin
    8
    ;; float64 _ymin
    8
    ;; float64 _xmax
    8
    ;; float64 _ymax
    8
    ;; float64 _zmin
    8
    ;; float64 _zmax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmin
       (sys::poke _xmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymin
       (sys::poke _ymin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _xmax
       (sys::poke _xmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymax
       (sys::poke _ymax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmin
       (sys::poke _zmin (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmax
       (sys::poke _zmax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmin
     (setq _xmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymin
     (setq _ymin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _xmax
     (setq _xmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymax
     (setq _ymax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmin
     (setq _zmin (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmax
     (setq _zmax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :md5sum-) "6e20dcf06c2cb2c3714e76bbf196e60c")
(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :datatype-) "gb_visual_detection_3d_msgs/BoundingBox3d")
(setf (get gb_visual_detection_3d_msgs::BoundingBox3d :definition-)
      "string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax

")



(provide :gb_visual_detection_3d_msgs/BoundingBox3d "6e20dcf06c2cb2c3714e76bbf196e60c")


