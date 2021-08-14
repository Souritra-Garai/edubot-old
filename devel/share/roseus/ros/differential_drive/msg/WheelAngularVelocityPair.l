;; Auto-generated. Do not edit!


(when (boundp 'differential_drive::WheelAngularVelocityPair)
  (if (not (find-package "DIFFERENTIAL_DRIVE"))
    (make-package "DIFFERENTIAL_DRIVE"))
  (shadow 'WheelAngularVelocityPair (find-package "DIFFERENTIAL_DRIVE")))
(unless (find-package "DIFFERENTIAL_DRIVE::WHEELANGULARVELOCITYPAIR")
  (make-package "DIFFERENTIAL_DRIVE::WHEELANGULARVELOCITYPAIR"))

(in-package "ROS")
;;//! \htmlinclude WheelAngularVelocityPair.msg.html


(defclass differential_drive::WheelAngularVelocityPair
  :super ros::object
  :slots (_wheel_angular_velocity_left _wheel_angular_velocity_right ))

(defmethod differential_drive::WheelAngularVelocityPair
  (:init
   (&key
    ((:wheel_angular_velocity_left __wheel_angular_velocity_left) 0.0)
    ((:wheel_angular_velocity_right __wheel_angular_velocity_right) 0.0)
    )
   (send-super :init)
   (setq _wheel_angular_velocity_left (float __wheel_angular_velocity_left))
   (setq _wheel_angular_velocity_right (float __wheel_angular_velocity_right))
   self)
  (:wheel_angular_velocity_left
   (&optional __wheel_angular_velocity_left)
   (if __wheel_angular_velocity_left (setq _wheel_angular_velocity_left __wheel_angular_velocity_left)) _wheel_angular_velocity_left)
  (:wheel_angular_velocity_right
   (&optional __wheel_angular_velocity_right)
   (if __wheel_angular_velocity_right (setq _wheel_angular_velocity_right __wheel_angular_velocity_right)) _wheel_angular_velocity_right)
  (:serialization-length
   ()
   (+
    ;; float32 _wheel_angular_velocity_left
    4
    ;; float32 _wheel_angular_velocity_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _wheel_angular_velocity_left
       (sys::poke _wheel_angular_velocity_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wheel_angular_velocity_right
       (sys::poke _wheel_angular_velocity_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _wheel_angular_velocity_left
     (setq _wheel_angular_velocity_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wheel_angular_velocity_right
     (setq _wheel_angular_velocity_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get differential_drive::WheelAngularVelocityPair :md5sum-) "adff221a07855e72470c2f5460fcf2d6")
(setf (get differential_drive::WheelAngularVelocityPair :datatype-) "differential_drive/WheelAngularVelocityPair")
(setf (get differential_drive::WheelAngularVelocityPair :definition-)
      "float32 wheel_angular_velocity_left
float32 wheel_angular_velocity_right
")



(provide :differential_drive/WheelAngularVelocityPair "adff221a07855e72470c2f5460fcf2d6")


