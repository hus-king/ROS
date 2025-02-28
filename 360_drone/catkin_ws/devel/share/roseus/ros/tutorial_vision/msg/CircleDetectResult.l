;; Auto-generated. Do not edit!


(when (boundp 'tutorial_vision::CircleDetectResult)
  (if (not (find-package "TUTORIAL_VISION"))
    (make-package "TUTORIAL_VISION"))
  (shadow 'CircleDetectResult (find-package "TUTORIAL_VISION")))
(unless (find-package "TUTORIAL_VISION::CIRCLEDETECTRESULT")
  (make-package "TUTORIAL_VISION::CIRCLEDETECTRESULT"))

(in-package "ROS")
;;//! \htmlinclude CircleDetectResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tutorial_vision::CircleDetectResult
  :super ros::object
  :slots (_header _height _width _circles ))

(defmethod tutorial_vision::CircleDetectResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:height __height) 0)
    ((:width __width) 0)
    ((:circles __circles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _circles __circles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:circles
   (&rest __circles)
   (if (keywordp (car __circles))
       (send* _circles __circles)
     (progn
       (if __circles (setq _circles (car __circles)))
       _circles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ;; tutorial_vision/CircleInfo[] _circles
    (apply #'+ (send-all _circles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;; tutorial_vision/CircleInfo[] _circles
     (write-long (length _circles) s)
     (dolist (elem _circles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; tutorial_vision/CircleInfo[] _circles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circles (let (r) (dotimes (i n) (push (instance tutorial_vision::CircleInfo :init) r)) r))
     (dolist (elem- _circles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tutorial_vision::CircleDetectResult :md5sum-) "920a6314571c6b3ce53335defc76681a")
(setf (get tutorial_vision::CircleDetectResult :datatype-) "tutorial_vision/CircleDetectResult")
(setf (get tutorial_vision::CircleDetectResult :definition-)
      "std_msgs/Header header
uint32 height
uint32 width
tutorial_vision/CircleInfo[] circles


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
MSG: tutorial_vision/CircleInfo
float64 center_x
float64 center_y
float64 radius

")



(provide :tutorial_vision/CircleDetectResult "920a6314571c6b3ce53335defc76681a")


