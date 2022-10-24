;; Auto-generated. Do not edit!


(when (boundp 'sensor_simulation::calibrate)
  (if (not (find-package "SENSOR_SIMULATION"))
    (make-package "SENSOR_SIMULATION"))
  (shadow 'calibrate (find-package "SENSOR_SIMULATION")))
(unless (find-package "SENSOR_SIMULATION::CALIBRATE")
  (make-package "SENSOR_SIMULATION::CALIBRATE"))
(unless (find-package "SENSOR_SIMULATION::CALIBRATEREQUEST")
  (make-package "SENSOR_SIMULATION::CALIBRATEREQUEST"))
(unless (find-package "SENSOR_SIMULATION::CALIBRATERESPONSE")
  (make-package "SENSOR_SIMULATION::CALIBRATERESPONSE"))

(in-package "ROS")





(defclass sensor_simulation::calibrateRequest
  :super ros::object
  :slots (_zero ))

(defmethod sensor_simulation::calibrateRequest
  (:init
   (&key
    ((:zero __zero) nil)
    )
   (send-super :init)
   (setq _zero __zero)
   self)
  (:zero
   (&optional __zero)
   (if __zero (setq _zero __zero)) _zero)
  (:serialization-length
   ()
   (+
    ;; bool _zero
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _zero
       (if _zero (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _zero
     (setq _zero (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sensor_simulation::calibrateResponse
  :super ros::object
  :slots (_baseline ))

(defmethod sensor_simulation::calibrateResponse
  (:init
   (&key
    ((:baseline __baseline) 0.0)
    )
   (send-super :init)
   (setq _baseline (float __baseline))
   self)
  (:baseline
   (&optional __baseline)
   (if __baseline (setq _baseline __baseline)) _baseline)
  (:serialization-length
   ()
   (+
    ;; float64 _baseline
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _baseline
       (sys::poke _baseline (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _baseline
     (setq _baseline (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass sensor_simulation::calibrate
  :super ros::object
  :slots ())

(setf (get sensor_simulation::calibrate :md5sum-) "75b28b77c2f551fa5005ce2f520330b5")
(setf (get sensor_simulation::calibrate :datatype-) "sensor_simulation/calibrate")
(setf (get sensor_simulation::calibrate :request) sensor_simulation::calibrateRequest)
(setf (get sensor_simulation::calibrate :response) sensor_simulation::calibrateResponse)

(defmethod sensor_simulation::calibrateRequest
  (:response () (instance sensor_simulation::calibrateResponse :init)))

(setf (get sensor_simulation::calibrateRequest :md5sum-) "75b28b77c2f551fa5005ce2f520330b5")
(setf (get sensor_simulation::calibrateRequest :datatype-) "sensor_simulation/calibrateRequest")
(setf (get sensor_simulation::calibrateRequest :definition-)
      "bool zero
---
float64 baseline
")

(setf (get sensor_simulation::calibrateResponse :md5sum-) "75b28b77c2f551fa5005ce2f520330b5")
(setf (get sensor_simulation::calibrateResponse :datatype-) "sensor_simulation/calibrateResponse")
(setf (get sensor_simulation::calibrateResponse :definition-)
      "bool zero
---
float64 baseline
")



(provide :sensor_simulation/calibrate "75b28b77c2f551fa5005ce2f520330b5")


