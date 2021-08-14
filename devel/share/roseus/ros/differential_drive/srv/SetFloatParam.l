;; Auto-generated. Do not edit!


(when (boundp 'differential_drive::SetFloatParam)
  (if (not (find-package "DIFFERENTIAL_DRIVE"))
    (make-package "DIFFERENTIAL_DRIVE"))
  (shadow 'SetFloatParam (find-package "DIFFERENTIAL_DRIVE")))
(unless (find-package "DIFFERENTIAL_DRIVE::SETFLOATPARAM")
  (make-package "DIFFERENTIAL_DRIVE::SETFLOATPARAM"))
(unless (find-package "DIFFERENTIAL_DRIVE::SETFLOATPARAMREQUEST")
  (make-package "DIFFERENTIAL_DRIVE::SETFLOATPARAMREQUEST"))
(unless (find-package "DIFFERENTIAL_DRIVE::SETFLOATPARAMRESPONSE")
  (make-package "DIFFERENTIAL_DRIVE::SETFLOATPARAMRESPONSE"))

(in-package "ROS")





(defclass differential_drive::SetFloatParamRequest
  :super ros::object
  :slots (_val ))

(defmethod differential_drive::SetFloatParamRequest
  (:init
   (&key
    ((:val __val) 0.0)
    )
   (send-super :init)
   (setq _val (float __val))
   self)
  (:val
   (&optional __val)
   (if __val (setq _val __val)) _val)
  (:serialization-length
   ()
   (+
    ;; float32 _val
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _val
       (sys::poke _val (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _val
     (setq _val (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass differential_drive::SetFloatParamResponse
  :super ros::object
  :slots ())

(defmethod differential_drive::SetFloatParamResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass differential_drive::SetFloatParam
  :super ros::object
  :slots ())

(setf (get differential_drive::SetFloatParam :md5sum-) "c9ee899b5f0899afa6060c9ba611652c")
(setf (get differential_drive::SetFloatParam :datatype-) "differential_drive/SetFloatParam")
(setf (get differential_drive::SetFloatParam :request) differential_drive::SetFloatParamRequest)
(setf (get differential_drive::SetFloatParam :response) differential_drive::SetFloatParamResponse)

(defmethod differential_drive::SetFloatParamRequest
  (:response () (instance differential_drive::SetFloatParamResponse :init)))

(setf (get differential_drive::SetFloatParamRequest :md5sum-) "c9ee899b5f0899afa6060c9ba611652c")
(setf (get differential_drive::SetFloatParamRequest :datatype-) "differential_drive/SetFloatParamRequest")
(setf (get differential_drive::SetFloatParamRequest :definition-)
      "float32 val
---

")

(setf (get differential_drive::SetFloatParamResponse :md5sum-) "c9ee899b5f0899afa6060c9ba611652c")
(setf (get differential_drive::SetFloatParamResponse :datatype-) "differential_drive/SetFloatParamResponse")
(setf (get differential_drive::SetFloatParamResponse :definition-)
      "float32 val
---

")



(provide :differential_drive/SetFloatParam "c9ee899b5f0899afa6060c9ba611652c")


