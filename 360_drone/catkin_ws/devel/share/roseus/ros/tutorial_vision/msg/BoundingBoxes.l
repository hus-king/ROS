;; Auto-generated. Do not edit!


(when (boundp 'tutorial_vision::BoundingBoxes)
  (if (not (find-package "TUTORIAL_VISION"))
    (make-package "TUTORIAL_VISION"))
  (shadow 'BoundingBoxes (find-package "TUTORIAL_VISION")))
(unless (find-package "TUTORIAL_VISION::BOUNDINGBOXES")
  (make-package "TUTORIAL_VISION::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tutorial_vision::BoundingBoxes
  :super ros::object
  :slots (_header _bounding_boxes ))

(defmethod tutorial_vision::BoundingBoxes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bounding_boxes __bounding_boxes) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _bounding_boxes __bounding_boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tutorial_vision/BoundingBox[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tutorial_vision/BoundingBox[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tutorial_vision/BoundingBox[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance tutorial_vision::BoundingBox :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tutorial_vision::BoundingBoxes :md5sum-) "0c66b76f3c0a60b31a7bca182a3b5d0d")
(setf (get tutorial_vision::BoundingBoxes :datatype-) "tutorial_vision/BoundingBoxes")
(setf (get tutorial_vision::BoundingBoxes :definition-)
      "Header header
BoundingBox[] bounding_boxes
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
MSG: tutorial_vision/BoundingBox
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
string Class
")



(provide :tutorial_vision/BoundingBoxes "0c66b76f3c0a60b31a7bca182a3b5d0d")


