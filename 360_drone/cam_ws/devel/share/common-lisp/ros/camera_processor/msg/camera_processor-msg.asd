
(cl:in-package :asdf)

(defsystem "camera_processor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PointDepth" :depends-on ("_package_PointDepth"))
    (:file "_package_PointDepth" :depends-on ("_package"))
  ))