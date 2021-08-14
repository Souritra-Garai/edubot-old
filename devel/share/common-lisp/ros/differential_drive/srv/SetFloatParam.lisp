; Auto-generated. Do not edit!


(cl:in-package differential_drive-srv)


;//! \htmlinclude SetFloatParam-request.msg.html

(cl:defclass <SetFloatParam-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetFloatParam-request (<SetFloatParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloatParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloatParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-srv:<SetFloatParam-request> is deprecated: use differential_drive-srv:SetFloatParam-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <SetFloatParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_drive-srv:val-val is deprecated.  Use differential_drive-srv:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloatParam-request>) ostream)
  "Serializes a message object of type '<SetFloatParam-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloatParam-request>) istream)
  "Deserializes a message object of type '<SetFloatParam-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'val) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloatParam-request>)))
  "Returns string type for a service object of type '<SetFloatParam-request>"
  "differential_drive/SetFloatParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloatParam-request)))
  "Returns string type for a service object of type 'SetFloatParam-request"
  "differential_drive/SetFloatParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloatParam-request>)))
  "Returns md5sum for a message object of type '<SetFloatParam-request>"
  "c9ee899b5f0899afa6060c9ba611652c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloatParam-request)))
  "Returns md5sum for a message object of type 'SetFloatParam-request"
  "c9ee899b5f0899afa6060c9ba611652c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloatParam-request>)))
  "Returns full string definition for message of type '<SetFloatParam-request>"
  (cl:format cl:nil "float32 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloatParam-request)))
  "Returns full string definition for message of type 'SetFloatParam-request"
  (cl:format cl:nil "float32 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloatParam-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloatParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloatParam-request
    (cl:cons ':val (val msg))
))
;//! \htmlinclude SetFloatParam-response.msg.html

(cl:defclass <SetFloatParam-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetFloatParam-response (<SetFloatParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloatParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloatParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_drive-srv:<SetFloatParam-response> is deprecated: use differential_drive-srv:SetFloatParam-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloatParam-response>) ostream)
  "Serializes a message object of type '<SetFloatParam-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloatParam-response>) istream)
  "Deserializes a message object of type '<SetFloatParam-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloatParam-response>)))
  "Returns string type for a service object of type '<SetFloatParam-response>"
  "differential_drive/SetFloatParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloatParam-response)))
  "Returns string type for a service object of type 'SetFloatParam-response"
  "differential_drive/SetFloatParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloatParam-response>)))
  "Returns md5sum for a message object of type '<SetFloatParam-response>"
  "c9ee899b5f0899afa6060c9ba611652c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloatParam-response)))
  "Returns md5sum for a message object of type 'SetFloatParam-response"
  "c9ee899b5f0899afa6060c9ba611652c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloatParam-response>)))
  "Returns full string definition for message of type '<SetFloatParam-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloatParam-response)))
  "Returns full string definition for message of type 'SetFloatParam-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloatParam-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloatParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloatParam-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFloatParam)))
  'SetFloatParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFloatParam)))
  'SetFloatParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloatParam)))
  "Returns string type for a service object of type '<SetFloatParam>"
  "differential_drive/SetFloatParam")