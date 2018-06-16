; Auto-generated. Do not edit!


(cl:in-package snake_control-msg)


;//! \htmlinclude snake_head_rel_pos.msg.html

(cl:defclass <snake_head_rel_pos> (roslisp-msg-protocol:ros-message)
  ((x_rel
    :reader x_rel
    :initarg :x_rel
    :type cl:float
    :initform 0.0)
   (y_rel
    :reader y_rel
    :initarg :y_rel
    :type cl:float
    :initform 0.0))
)

(cl:defclass snake_head_rel_pos (<snake_head_rel_pos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <snake_head_rel_pos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'snake_head_rel_pos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snake_control-msg:<snake_head_rel_pos> is deprecated: use snake_control-msg:snake_head_rel_pos instead.")))

(cl:ensure-generic-function 'x_rel-val :lambda-list '(m))
(cl:defmethod x_rel-val ((m <snake_head_rel_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snake_control-msg:x_rel-val is deprecated.  Use snake_control-msg:x_rel instead.")
  (x_rel m))

(cl:ensure-generic-function 'y_rel-val :lambda-list '(m))
(cl:defmethod y_rel-val ((m <snake_head_rel_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snake_control-msg:y_rel-val is deprecated.  Use snake_control-msg:y_rel instead.")
  (y_rel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <snake_head_rel_pos>) ostream)
  "Serializes a message object of type '<snake_head_rel_pos>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <snake_head_rel_pos>) istream)
  "Deserializes a message object of type '<snake_head_rel_pos>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_rel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_rel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<snake_head_rel_pos>)))
  "Returns string type for a message object of type '<snake_head_rel_pos>"
  "snake_control/snake_head_rel_pos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'snake_head_rel_pos)))
  "Returns string type for a message object of type 'snake_head_rel_pos"
  "snake_control/snake_head_rel_pos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<snake_head_rel_pos>)))
  "Returns md5sum for a message object of type '<snake_head_rel_pos>"
  "deda16f7231e7eaf16efd16c6f2840d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'snake_head_rel_pos)))
  "Returns md5sum for a message object of type 'snake_head_rel_pos"
  "deda16f7231e7eaf16efd16c6f2840d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<snake_head_rel_pos>)))
  "Returns full string definition for message of type '<snake_head_rel_pos>"
  (cl:format cl:nil "float64 x_rel~%float64 y_rel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'snake_head_rel_pos)))
  "Returns full string definition for message of type 'snake_head_rel_pos"
  (cl:format cl:nil "float64 x_rel~%float64 y_rel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <snake_head_rel_pos>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <snake_head_rel_pos>))
  "Converts a ROS message object to a list"
  (cl:list 'snake_head_rel_pos
    (cl:cons ':x_rel (x_rel msg))
    (cl:cons ':y_rel (y_rel msg))
))
