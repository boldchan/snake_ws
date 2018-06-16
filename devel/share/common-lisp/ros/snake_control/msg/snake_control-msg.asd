
(cl:in-package :asdf)

(defsystem "snake_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "snake_head_rel_pos" :depends-on ("_package_snake_head_rel_pos"))
    (:file "_package_snake_head_rel_pos" :depends-on ("_package"))
  ))