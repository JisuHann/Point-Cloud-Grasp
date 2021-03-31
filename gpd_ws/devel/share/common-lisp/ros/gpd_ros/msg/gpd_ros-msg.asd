
(cl:in-package :asdf)

(defsystem "gpd_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CloudIndexed" :depends-on ("_package_CloudIndexed"))
    (:file "_package_CloudIndexed" :depends-on ("_package"))
    (:file "CloudSamples" :depends-on ("_package_CloudSamples"))
    (:file "_package_CloudSamples" :depends-on ("_package"))
    (:file "CloudSources" :depends-on ("_package_CloudSources"))
    (:file "_package_CloudSources" :depends-on ("_package"))
    (:file "GraspConfig" :depends-on ("_package_GraspConfig"))
    (:file "_package_GraspConfig" :depends-on ("_package"))
    (:file "GraspConfigList" :depends-on ("_package_GraspConfigList"))
    (:file "_package_GraspConfigList" :depends-on ("_package"))
    (:file "SamplesMsg" :depends-on ("_package_SamplesMsg"))
    (:file "_package_SamplesMsg" :depends-on ("_package"))
  ))