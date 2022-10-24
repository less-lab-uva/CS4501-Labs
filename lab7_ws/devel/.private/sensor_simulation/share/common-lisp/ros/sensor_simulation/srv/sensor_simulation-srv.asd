
(cl:in-package :asdf)

(defsystem "sensor_simulation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "calibrate" :depends-on ("_package_calibrate"))
    (:file "_package_calibrate" :depends-on ("_package"))
  ))