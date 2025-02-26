; Auto-generated. Do not edit!


(cl:in-package tutorial_vision-msg)


;//! \htmlinclude CircleDetectResult.msg.html

(cl:defclass <CircleDetectResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (circles
    :reader circles
    :initarg :circles
    :type (cl:vector tutorial_vision-msg:CircleInfo)
   :initform (cl:make-array 0 :element-type 'tutorial_vision-msg:CircleInfo :initial-element (cl:make-instance 'tutorial_vision-msg:CircleInfo))))
)

(cl:defclass CircleDetectResult (<CircleDetectResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CircleDetectResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CircleDetectResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tutorial_vision-msg:<CircleDetectResult> is deprecated: use tutorial_vision-msg:CircleDetectResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CircleDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:header-val is deprecated.  Use tutorial_vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CircleDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:height-val is deprecated.  Use tutorial_vision-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CircleDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:width-val is deprecated.  Use tutorial_vision-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'circles-val :lambda-list '(m))
(cl:defmethod circles-val ((m <CircleDetectResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:circles-val is deprecated.  Use tutorial_vision-msg:circles instead.")
  (circles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CircleDetectResult>) ostream)
  "Serializes a message object of type '<CircleDetectResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'circles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'circles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CircleDetectResult>) istream)
  "Deserializes a message object of type '<CircleDetectResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'circles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'circles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tutorial_vision-msg:CircleInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CircleDetectResult>)))
  "Returns string type for a message object of type '<CircleDetectResult>"
  "tutorial_vision/CircleDetectResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CircleDetectResult)))
  "Returns string type for a message object of type 'CircleDetectResult"
  "tutorial_vision/CircleDetectResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CircleDetectResult>)))
  "Returns md5sum for a message object of type '<CircleDetectResult>"
  "920a6314571c6b3ce53335defc76681a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CircleDetectResult)))
  "Returns md5sum for a message object of type 'CircleDetectResult"
  "920a6314571c6b3ce53335defc76681a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CircleDetectResult>)))
  "Returns full string definition for message of type '<CircleDetectResult>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 height~%uint32 width~%tutorial_vision/CircleInfo[] circles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tutorial_vision/CircleInfo~%float64 center_x~%float64 center_y~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CircleDetectResult)))
  "Returns full string definition for message of type 'CircleDetectResult"
  (cl:format cl:nil "std_msgs/Header header~%uint32 height~%uint32 width~%tutorial_vision/CircleInfo[] circles~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tutorial_vision/CircleInfo~%float64 center_x~%float64 center_y~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CircleDetectResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'circles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CircleDetectResult>))
  "Converts a ROS message object to a list"
  (cl:list 'CircleDetectResult
    (cl:cons ':header (header msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':circles (circles msg))
))
