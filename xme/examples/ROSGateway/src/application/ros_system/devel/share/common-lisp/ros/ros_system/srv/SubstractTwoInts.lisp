; Auto-generated. Do not edit!


(cl:in-package ros_system-srv)


;//! \htmlinclude SubstractTwoInts-request.msg.html

(cl:defclass <SubstractTwoInts-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass SubstractTwoInts-request (<SubstractTwoInts-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubstractTwoInts-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubstractTwoInts-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_system-srv:<SubstractTwoInts-request> is deprecated: use ros_system-srv:SubstractTwoInts-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <SubstractTwoInts-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_system-srv:a-val is deprecated.  Use ros_system-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <SubstractTwoInts-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_system-srv:b-val is deprecated.  Use ros_system-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubstractTwoInts-request>) ostream)
  "Serializes a message object of type '<SubstractTwoInts-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubstractTwoInts-request>) istream)
  "Deserializes a message object of type '<SubstractTwoInts-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubstractTwoInts-request>)))
  "Returns string type for a service object of type '<SubstractTwoInts-request>"
  "ros_system/SubstractTwoIntsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubstractTwoInts-request)))
  "Returns string type for a service object of type 'SubstractTwoInts-request"
  "ros_system/SubstractTwoIntsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubstractTwoInts-request>)))
  "Returns md5sum for a message object of type '<SubstractTwoInts-request>"
  "6261ad0e9a1d7803132e49e14b2897b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubstractTwoInts-request)))
  "Returns md5sum for a message object of type 'SubstractTwoInts-request"
  "6261ad0e9a1d7803132e49e14b2897b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubstractTwoInts-request>)))
  "Returns full string definition for message of type '<SubstractTwoInts-request>"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubstractTwoInts-request)))
  "Returns full string definition for message of type 'SubstractTwoInts-request"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubstractTwoInts-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubstractTwoInts-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SubstractTwoInts-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude SubstractTwoInts-response.msg.html

(cl:defclass <SubstractTwoInts-response> (roslisp-msg-protocol:ros-message)
  ((difference
    :reader difference
    :initarg :difference
    :type cl:integer
    :initform 0))
)

(cl:defclass SubstractTwoInts-response (<SubstractTwoInts-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SubstractTwoInts-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SubstractTwoInts-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_system-srv:<SubstractTwoInts-response> is deprecated: use ros_system-srv:SubstractTwoInts-response instead.")))

(cl:ensure-generic-function 'difference-val :lambda-list '(m))
(cl:defmethod difference-val ((m <SubstractTwoInts-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_system-srv:difference-val is deprecated.  Use ros_system-srv:difference instead.")
  (difference m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SubstractTwoInts-response>) ostream)
  "Serializes a message object of type '<SubstractTwoInts-response>"
  (cl:let* ((signed (cl:slot-value msg 'difference)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SubstractTwoInts-response>) istream)
  "Deserializes a message object of type '<SubstractTwoInts-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'difference) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SubstractTwoInts-response>)))
  "Returns string type for a service object of type '<SubstractTwoInts-response>"
  "ros_system/SubstractTwoIntsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubstractTwoInts-response)))
  "Returns string type for a service object of type 'SubstractTwoInts-response"
  "ros_system/SubstractTwoIntsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SubstractTwoInts-response>)))
  "Returns md5sum for a message object of type '<SubstractTwoInts-response>"
  "6261ad0e9a1d7803132e49e14b2897b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SubstractTwoInts-response)))
  "Returns md5sum for a message object of type 'SubstractTwoInts-response"
  "6261ad0e9a1d7803132e49e14b2897b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SubstractTwoInts-response>)))
  "Returns full string definition for message of type '<SubstractTwoInts-response>"
  (cl:format cl:nil "int64 difference~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SubstractTwoInts-response)))
  "Returns full string definition for message of type 'SubstractTwoInts-response"
  (cl:format cl:nil "int64 difference~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SubstractTwoInts-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SubstractTwoInts-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SubstractTwoInts-response
    (cl:cons ':difference (difference msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SubstractTwoInts)))
  'SubstractTwoInts-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SubstractTwoInts)))
  'SubstractTwoInts-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SubstractTwoInts)))
  "Returns string type for a service object of type '<SubstractTwoInts>"
  "ros_system/SubstractTwoInts")