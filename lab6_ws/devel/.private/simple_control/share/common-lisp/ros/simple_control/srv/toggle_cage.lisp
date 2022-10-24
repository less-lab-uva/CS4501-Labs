; Auto-generated. Do not edit!


(cl:in-package simple_control-srv)


;//! \htmlinclude toggle_cage-request.msg.html

(cl:defclass <toggle_cage-request> (roslisp-msg-protocol:ros-message)
  ((cage_on
    :reader cage_on
    :initarg :cage_on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass toggle_cage-request (<toggle_cage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <toggle_cage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'toggle_cage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_control-srv:<toggle_cage-request> is deprecated: use simple_control-srv:toggle_cage-request instead.")))

(cl:ensure-generic-function 'cage_on-val :lambda-list '(m))
(cl:defmethod cage_on-val ((m <toggle_cage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_control-srv:cage_on-val is deprecated.  Use simple_control-srv:cage_on instead.")
  (cage_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <toggle_cage-request>) ostream)
  "Serializes a message object of type '<toggle_cage-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cage_on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <toggle_cage-request>) istream)
  "Deserializes a message object of type '<toggle_cage-request>"
    (cl:setf (cl:slot-value msg 'cage_on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<toggle_cage-request>)))
  "Returns string type for a service object of type '<toggle_cage-request>"
  "simple_control/toggle_cageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'toggle_cage-request)))
  "Returns string type for a service object of type 'toggle_cage-request"
  "simple_control/toggle_cageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<toggle_cage-request>)))
  "Returns md5sum for a message object of type '<toggle_cage-request>"
  "e06196d4192ed08fc72383b1573c2db2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'toggle_cage-request)))
  "Returns md5sum for a message object of type 'toggle_cage-request"
  "e06196d4192ed08fc72383b1573c2db2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<toggle_cage-request>)))
  "Returns full string definition for message of type '<toggle_cage-request>"
  (cl:format cl:nil "bool cage_on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'toggle_cage-request)))
  "Returns full string definition for message of type 'toggle_cage-request"
  (cl:format cl:nil "bool cage_on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <toggle_cage-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <toggle_cage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'toggle_cage-request
    (cl:cons ':cage_on (cage_on msg))
))
;//! \htmlinclude toggle_cage-response.msg.html

(cl:defclass <toggle_cage-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass toggle_cage-response (<toggle_cage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <toggle_cage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'toggle_cage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_control-srv:<toggle_cage-response> is deprecated: use simple_control-srv:toggle_cage-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <toggle_cage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_control-srv:success-val is deprecated.  Use simple_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <toggle_cage-response>) ostream)
  "Serializes a message object of type '<toggle_cage-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <toggle_cage-response>) istream)
  "Deserializes a message object of type '<toggle_cage-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<toggle_cage-response>)))
  "Returns string type for a service object of type '<toggle_cage-response>"
  "simple_control/toggle_cageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'toggle_cage-response)))
  "Returns string type for a service object of type 'toggle_cage-response"
  "simple_control/toggle_cageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<toggle_cage-response>)))
  "Returns md5sum for a message object of type '<toggle_cage-response>"
  "e06196d4192ed08fc72383b1573c2db2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'toggle_cage-response)))
  "Returns md5sum for a message object of type 'toggle_cage-response"
  "e06196d4192ed08fc72383b1573c2db2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<toggle_cage-response>)))
  "Returns full string definition for message of type '<toggle_cage-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'toggle_cage-response)))
  "Returns full string definition for message of type 'toggle_cage-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <toggle_cage-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <toggle_cage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'toggle_cage-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'toggle_cage)))
  'toggle_cage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'toggle_cage)))
  'toggle_cage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'toggle_cage)))
  "Returns string type for a service object of type '<toggle_cage>"
  "simple_control/toggle_cage")