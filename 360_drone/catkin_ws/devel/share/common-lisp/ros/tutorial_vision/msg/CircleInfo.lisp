; Auto-generated. Do not edit!


(cl:in-package tutorial_vision-msg)


;//! \htmlinclude CircleInfo.msg.html

(cl:defclass <CircleInfo> (roslisp-msg-protocol:ros-message)
  ((center_x
    :reader center_x
    :initarg :center_x
    :type cl:float
    :initform 0.0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass CircleInfo (<CircleInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CircleInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CircleInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tutorial_vision-msg:<CircleInfo> is deprecated: use tutorial_vision-msg:CircleInfo instead.")))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <CircleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:center_x-val is deprecated.  Use tutorial_vision-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <CircleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:center_y-val is deprecated.  Use tutorial_vision-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <CircleInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorial_vision-msg:radius-val is deprecated.  Use tutorial_vision-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CircleInfo>) ostream)
  "Serializes a message object of type '<CircleInfo>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'center_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CircleInfo>) istream)
  "Deserializes a message object of type '<CircleInfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CircleInfo>)))
  "Returns string type for a message object of type '<CircleInfo>"
  "tutorial_vision/CircleInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CircleInfo)))
  "Returns string type for a message object of type 'CircleInfo"
  "tutorial_vision/CircleInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CircleInfo>)))
  "Returns md5sum for a message object of type '<CircleInfo>"
  "28a09b8c5ded5c4af6b7db31b0bee222")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CircleInfo)))
  "Returns md5sum for a message object of type 'CircleInfo"
  "28a09b8c5ded5c4af6b7db31b0bee222")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CircleInfo>)))
  "Returns full string definition for message of type '<CircleInfo>"
  (cl:format cl:nil "float64 center_x~%float64 center_y~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CircleInfo)))
  "Returns full string definition for message of type 'CircleInfo"
  (cl:format cl:nil "float64 center_x~%float64 center_y~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CircleInfo>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CircleInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'CircleInfo
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':radius (radius msg))
))
