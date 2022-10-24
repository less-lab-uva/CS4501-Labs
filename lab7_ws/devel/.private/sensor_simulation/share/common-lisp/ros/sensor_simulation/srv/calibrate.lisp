; Auto-generated. Do not edit!


(cl:in-package sensor_simulation-srv)


;//! \htmlinclude calibrate-request.msg.html

(cl:defclass <calibrate-request> (roslisp-msg-protocol:ros-message)
  ((zero
    :reader zero
    :initarg :zero
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass calibrate-request (<calibrate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calibrate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calibrate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_simulation-srv:<calibrate-request> is deprecated: use sensor_simulation-srv:calibrate-request instead.")))

(cl:ensure-generic-function 'zero-val :lambda-list '(m))
(cl:defmethod zero-val ((m <calibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_simulation-srv:zero-val is deprecated.  Use sensor_simulation-srv:zero instead.")
  (zero m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calibrate-request>) ostream)
  "Serializes a message object of type '<calibrate-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'zero) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calibrate-request>) istream)
  "Deserializes a message object of type '<calibrate-request>"
    (cl:setf (cl:slot-value msg 'zero) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calibrate-request>)))
  "Returns string type for a service object of type '<calibrate-request>"
  "sensor_simulation/calibrateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate-request)))
  "Returns string type for a service object of type 'calibrate-request"
  "sensor_simulation/calibrateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calibrate-request>)))
  "Returns md5sum for a message object of type '<calibrate-request>"
  "75b28b77c2f551fa5005ce2f520330b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calibrate-request)))
  "Returns md5sum for a message object of type 'calibrate-request"
  "75b28b77c2f551fa5005ce2f520330b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calibrate-request>)))
  "Returns full string definition for message of type '<calibrate-request>"
  (cl:format cl:nil "bool zero~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calibrate-request)))
  "Returns full string definition for message of type 'calibrate-request"
  (cl:format cl:nil "bool zero~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calibrate-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calibrate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'calibrate-request
    (cl:cons ':zero (zero msg))
))
;//! \htmlinclude calibrate-response.msg.html

(cl:defclass <calibrate-response> (roslisp-msg-protocol:ros-message)
  ((baseline
    :reader baseline
    :initarg :baseline
    :type cl:float
    :initform 0.0))
)

(cl:defclass calibrate-response (<calibrate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calibrate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calibrate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_simulation-srv:<calibrate-response> is deprecated: use sensor_simulation-srv:calibrate-response instead.")))

(cl:ensure-generic-function 'baseline-val :lambda-list '(m))
(cl:defmethod baseline-val ((m <calibrate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_simulation-srv:baseline-val is deprecated.  Use sensor_simulation-srv:baseline instead.")
  (baseline m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calibrate-response>) ostream)
  "Serializes a message object of type '<calibrate-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'baseline))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calibrate-response>) istream)
  "Deserializes a message object of type '<calibrate-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'baseline) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calibrate-response>)))
  "Returns string type for a service object of type '<calibrate-response>"
  "sensor_simulation/calibrateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate-response)))
  "Returns string type for a service object of type 'calibrate-response"
  "sensor_simulation/calibrateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calibrate-response>)))
  "Returns md5sum for a message object of type '<calibrate-response>"
  "75b28b77c2f551fa5005ce2f520330b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calibrate-response)))
  "Returns md5sum for a message object of type 'calibrate-response"
  "75b28b77c2f551fa5005ce2f520330b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calibrate-response>)))
  "Returns full string definition for message of type '<calibrate-response>"
  (cl:format cl:nil "float64 baseline~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calibrate-response)))
  "Returns full string definition for message of type 'calibrate-response"
  (cl:format cl:nil "float64 baseline~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calibrate-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calibrate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'calibrate-response
    (cl:cons ':baseline (baseline msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'calibrate)))
  'calibrate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'calibrate)))
  'calibrate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate)))
  "Returns string type for a service object of type '<calibrate>"
  "sensor_simulation/calibrate")