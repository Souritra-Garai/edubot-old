; Auto-generated. Do not edit!


(cl:in-package differential_drive-msg)


;//! \htmlinclude WheelAngularVelocityPair.msg.html

(cl:defclass <WheelAngularVelocityPair> (roslisp-msg-protocol:ros-message)
  ((wheel_angular_velocity_left
    :reader wheel_angular_velocity_left
    :initarg :wheel_angular_velocity_left
    :type cl:float
    :initform 0.0)
   (wheel_angular_velocity_right
    :reader wheel_angular_velocity_right
    :initarg :wheel_angular_velocity_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelAngularVelocityPair (<WheelAngularVelocityPair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelAngularVelocityPair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelAngularVelocityPair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-msg:<WheelAngularVelocityPair> is deprecated: use differential_drive-msg:WheelAngularVelocityPair instead.")))

(cl:ensure-generic-function 'wheel_angular_velocity_left-val :lambda-list '(m))
(cl:defmethod wheel_angular_velocity_left-val ((m <WheelAngularVelocityPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:wheel_angular_velocity_left-val is deprecated.  Use differential_drive-msg:wheel_angular_velocity_left instead.")
  (wheel_angular_velocity_left m))

(cl:ensure-generic-function 'wheel_angular_velocity_right-val :lambda-list '(m))
(cl:defmethod wheel_angular_velocity_right-val ((m <WheelAngularVelocityPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-msg:wheel_angular_velocity_right-val is deprecated.  Use differential_drive-msg:wheel_angular_velocity_right instead.")
  (wheel_angular_velocity_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelAngularVelocityPair>) ostream)
  "Serializes a message object of type '<WheelAngularVelocityPair>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wheel_angular_velocity_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wheel_angular_velocity_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelAngularVelocityPair>) istream)
  "Deserializes a message object of type '<WheelAngularVelocityPair>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_angular_velocity_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_angular_velocity_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelAngularVelocityPair>)))
  "Returns string type for a message object of type '<WheelAngularVelocityPair>"
  "differential_drive/WheelAngularVelocityPair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelAngularVelocityPair)))
  "Returns string type for a message object of type 'WheelAngularVelocityPair"
  "differential_drive/WheelAngularVelocityPair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelAngularVelocityPair>)))
  "Returns md5sum for a message object of type '<WheelAngularVelocityPair>"
  "adff221a07855e72470c2f5460fcf2d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelAngularVelocityPair)))
  "Returns md5sum for a message object of type 'WheelAngularVelocityPair"
  "adff221a07855e72470c2f5460fcf2d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelAngularVelocityPair>)))
  "Returns full string definition for message of type '<WheelAngularVelocityPair>"
  (cl:format cl:nil "float32 wheel_angular_velocity_left~%float32 wheel_angular_velocity_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelAngularVelocityPair)))
  "Returns full string definition for message of type 'WheelAngularVelocityPair"
  (cl:format cl:nil "float32 wheel_angular_velocity_left~%float32 wheel_angular_velocity_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelAngularVelocityPair>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelAngularVelocityPair>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelAngularVelocityPair
    (cl:cons ':wheel_angular_velocity_left (wheel_angular_velocity_left msg))
    (cl:cons ':wheel_angular_velocity_right (wheel_angular_velocity_right msg))
))
