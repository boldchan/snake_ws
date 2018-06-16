;; Auto-generated. Do not edit!


(when (boundp 'snake_control::snake_head_rel_pos)
  (if (not (find-package "SNAKE_CONTROL"))
    (make-package "SNAKE_CONTROL"))
  (shadow 'snake_head_rel_pos (find-package "SNAKE_CONTROL")))
(unless (find-package "SNAKE_CONTROL::SNAKE_HEAD_REL_POS")
  (make-package "SNAKE_CONTROL::SNAKE_HEAD_REL_POS"))

(in-package "ROS")
;;//! \htmlinclude snake_head_rel_pos.msg.html


(defclass snake_control::snake_head_rel_pos
  :super ros::object
  :slots (_x_rel _y_rel ))

(defmethod snake_control::snake_head_rel_pos
  (:init
   (&key
    ((:x_rel __x_rel) 0.0)
    ((:y_rel __y_rel) 0.0)
    )
   (send-super :init)
   (setq _x_rel (float __x_rel))
   (setq _y_rel (float __y_rel))
   self)
  (:x_rel
   (&optional __x_rel)
   (if __x_rel (setq _x_rel __x_rel)) _x_rel)
  (:y_rel
   (&optional __y_rel)
   (if __y_rel (setq _y_rel __y_rel)) _y_rel)
  (:serialization-length
   ()
   (+
    ;; float64 _x_rel
    8
    ;; float64 _y_rel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_rel
       (sys::poke _x_rel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_rel
       (sys::poke _y_rel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_rel
     (setq _x_rel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_rel
     (setq _y_rel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get snake_control::snake_head_rel_pos :md5sum-) "deda16f7231e7eaf16efd16c6f2840d9")
(setf (get snake_control::snake_head_rel_pos :datatype-) "snake_control/snake_head_rel_pos")
(setf (get snake_control::snake_head_rel_pos :definition-)
      "float64 x_rel
float64 y_rel

")



(provide :snake_control/snake_head_rel_pos "deda16f7231e7eaf16efd16c6f2840d9")


