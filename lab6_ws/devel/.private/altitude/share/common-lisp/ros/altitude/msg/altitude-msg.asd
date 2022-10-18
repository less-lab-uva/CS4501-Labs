
(cl:in-package :asdf)

(defsystem "altitude-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AltitudeStamped" :depends-on ("_package_AltitudeStamped"))
    (:file "_package_AltitudeStamped" :depends-on ("_package"))
  ))