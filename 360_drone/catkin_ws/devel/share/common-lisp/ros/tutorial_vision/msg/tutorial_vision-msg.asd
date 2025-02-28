
(cl:in-package :asdf)

(defsystem "tutorial_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBoxes" :depends-on ("_package_BoundingBoxes"))
    (:file "_package_BoundingBoxes" :depends-on ("_package"))
    (:file "CircleDetectResult" :depends-on ("_package_CircleDetectResult"))
    (:file "_package_CircleDetectResult" :depends-on ("_package"))
    (:file "CircleInfo" :depends-on ("_package_CircleInfo"))
    (:file "_package_CircleInfo" :depends-on ("_package"))
    (:file "ObjectCount" :depends-on ("_package_ObjectCount"))
    (:file "_package_ObjectCount" :depends-on ("_package"))
    (:file "StringStamped" :depends-on ("_package_StringStamped"))
    (:file "_package_StringStamped" :depends-on ("_package"))
  ))