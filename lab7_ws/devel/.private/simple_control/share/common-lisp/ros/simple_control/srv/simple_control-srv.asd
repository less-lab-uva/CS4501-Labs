
(cl:in-package :asdf)

(defsystem "simple_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "toggle_cage" :depends-on ("_package_toggle_cage"))
    (:file "_package_toggle_cage" :depends-on ("_package"))
  ))