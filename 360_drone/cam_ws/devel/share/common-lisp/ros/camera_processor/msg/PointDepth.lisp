; Auto-generated. Do not edit!


(cl:in-package camera_processor-msg)


;//! \htmlinclude PointDepth.msg.html

(cl:defclass <PointDepth> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (depths
    :reader depths
    :initarg :depths
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PointDepth (<PointDepth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointDepth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointDepth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_processor-msg:<PointDepth> is deprecated: use camera_processor-msg:PointDepth instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PointDepth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_processor-msg:header-val is deprecated.  Use camera_processor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'depths-val :lambda-list '(m))
(cl:defmethod depths-val ((m <PointDepth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_processor-msg:depths-val is deprecated.  Use camera_processor-msg:depths instead.")
  (depths m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointDepth>) ostream)
  "Serializes a message object of type '<PointDepth>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'depths))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'depths))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointDepth>) istream)
  "Deserializes a message object of type '<PointDepth>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'depths) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'depths)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointDepth>)))
  "Returns string type for a message object of type '<PointDepth>"
  "camera_processor/PointDepth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointDepth)))
  "Returns string type for a message object of type 'PointDepth"
  "camera_processor/PointDepth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointDepth>)))
  "Returns md5sum for a message object of type '<PointDepth>"
  "9458e1769307167ea54afd5c9deb9fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointDepth)))
  "Returns md5sum for a message object of type 'PointDepth"
  "9458e1769307167ea54afd5c9deb9fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointDepth>)))
  "Returns full string definition for message of type '<PointDepth>"
  (cl:format cl:nil "Header header~%float32[] depths~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointDepth)))
  "Returns full string definition for message of type 'PointDepth"
  (cl:format cl:nil "Header header~%float32[] depths~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointDepth>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'depths) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointDepth>))
  "Converts a ROS message object to a list"
  (cl:list 'PointDepth
    (cl:cons ':header (header msg))
    (cl:cons ':depths (depths msg))
))
