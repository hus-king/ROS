;; Auto-generated. Do not edit!


(when (boundp 'camera_processor::PointDepth)
  (if (not (find-package "CAMERA_PROCESSOR"))
    (make-package "CAMERA_PROCESSOR"))
  (shadow 'PointDepth (find-package "CAMERA_PROCESSOR")))
(unless (find-package "CAMERA_PROCESSOR::POINTDEPTH")
  (make-package "CAMERA_PROCESSOR::POINTDEPTH"))

(in-package "ROS")
;;//! \htmlinclude PointDepth.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass camera_processor::PointDepth
  :super ros::object
  :slots (_header _depths ))

(defmethod camera_processor::PointDepth
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:depths __depths) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _depths __depths)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:depths
   (&optional __depths)
   (if __depths (setq _depths __depths)) _depths)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _depths
    (* 4    (length _depths)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _depths
     (write-long (length _depths) s)
     (dotimes (i (length _depths))
       (sys::poke (elt _depths i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _depths
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _depths (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _depths i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get camera_processor::PointDepth :md5sum-) "9458e1769307167ea54afd5c9deb9fc4")
(setf (get camera_processor::PointDepth :datatype-) "camera_processor/PointDepth")
(setf (get camera_processor::PointDepth :definition-)
      "Header header
float32[] depths

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

")



(provide :camera_processor/PointDepth "9458e1769307167ea54afd5c9deb9fc4")


