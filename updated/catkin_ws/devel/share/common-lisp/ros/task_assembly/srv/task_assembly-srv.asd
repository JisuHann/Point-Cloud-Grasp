
(cl:in-package :asdf)

(defsystem "task_assembly-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "door_open_planner" :depends-on ("_package_door_open_planner"))
    (:file "_package_door_open_planner" :depends-on ("_package"))
  ))