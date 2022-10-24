;; Auto-generated. Do not edit!


(when (boundp 'simple_control::toggle_cage)
  (if (not (find-package "SIMPLE_CONTROL"))
    (make-package "SIMPLE_CONTROL"))
  (shadow 'toggle_cage (find-package "SIMPLE_CONTROL")))
(unless (find-package "SIMPLE_CONTROL::TOGGLE_CAGE")
  (make-package "SIMPLE_CONTROL::TOGGLE_CAGE"))
(unless (find-package "SIMPLE_CONTROL::TOGGLE_CAGEREQUEST")
  (make-package "SIMPLE_CONTROL::TOGGLE_CAGEREQUEST"))
(unless (find-package "SIMPLE_CONTROL::TOGGLE_CAGERESPONSE")
  (make-package "SIMPLE_CONTROL::TOGGLE_CAGERESPONSE"))

(in-package "ROS")





(defclass simple_control::toggle_cageRequest
  :super ros::object
  :slots (_cage_on ))

(defmethod simple_control::toggle_cageRequest
  (:init
   (&key
    ((:cage_on __cage_on) nil)
    )
   (send-super :init)
   (setq _cage_on __cage_on)
   self)
  (:cage_on
   (&optional __cage_on)
   (if __cage_on (setq _cage_on __cage_on)) _cage_on)
  (:serialization-length
   ()
   (+
    ;; bool _cage_on
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _cage_on
       (if _cage_on (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _cage_on
     (setq _cage_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass simple_control::toggle_cageResponse
  :super ros::object
  :slots (_success ))

(defmethod simple_control::toggle_cageResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass simple_control::toggle_cage
  :super ros::object
  :slots ())

(setf (get simple_control::toggle_cage :md5sum-) "e06196d4192ed08fc72383b1573c2db2")
(setf (get simple_control::toggle_cage :datatype-) "simple_control/toggle_cage")
(setf (get simple_control::toggle_cage :request) simple_control::toggle_cageRequest)
(setf (get simple_control::toggle_cage :response) simple_control::toggle_cageResponse)

(defmethod simple_control::toggle_cageRequest
  (:response () (instance simple_control::toggle_cageResponse :init)))

(setf (get simple_control::toggle_cageRequest :md5sum-) "e06196d4192ed08fc72383b1573c2db2")
(setf (get simple_control::toggle_cageRequest :datatype-) "simple_control/toggle_cageRequest")
(setf (get simple_control::toggle_cageRequest :definition-)
      "bool cage_on
---
bool success
")

(setf (get simple_control::toggle_cageResponse :md5sum-) "e06196d4192ed08fc72383b1573c2db2")
(setf (get simple_control::toggle_cageResponse :datatype-) "simple_control/toggle_cageResponse")
(setf (get simple_control::toggle_cageResponse :definition-)
      "bool cage_on
---
bool success
")



(provide :simple_control/toggle_cage "e06196d4192ed08fc72383b1573c2db2")


