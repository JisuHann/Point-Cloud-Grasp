;; Auto-generated. Do not edit!


(when (boundp 'task_assembly::target)
  (if (not (find-package "TASK_ASSEMBLY"))
    (make-package "TASK_ASSEMBLY"))
  (shadow 'target (find-package "TASK_ASSEMBLY")))
(unless (find-package "TASK_ASSEMBLY::TARGET")
  (make-package "TASK_ASSEMBLY::TARGET"))

(in-package "ROS")
;;//! \htmlinclude target.msg.html


(defclass task_assembly::target
  :super ros::object
  :slots (_congfig_target ))

(defmethod task_assembly::target
  (:init
   (&key
    ((:congfig_target __congfig_target) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _congfig_target __congfig_target)
   self)
  (:congfig_target
   (&optional __congfig_target)
   (if __congfig_target (setq _congfig_target __congfig_target)) _congfig_target)
  (:serialization-length
   ()
   (+
    ;; float64[] _congfig_target
    (* 8    (length _congfig_target)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _congfig_target
     (write-long (length _congfig_target) s)
     (dotimes (i (length _congfig_target))
       (sys::poke (elt _congfig_target i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _congfig_target
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _congfig_target (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _congfig_target i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get task_assembly::target :md5sum-) "f926a1e4c3de25050f5c6ca556f561b4")
(setf (get task_assembly::target :datatype-) "task_assembly/target")
(setf (get task_assembly::target :definition-)
      "float64[] congfig_target
")



(provide :task_assembly/target "f926a1e4c3de25050f5c6ca556f561b4")


