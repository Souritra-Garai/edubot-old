
(cl:in-package :asdf)

(defsystem "differential_drive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelAngularVelocityPair" :depends-on ("_package_WheelAngularVelocityPair"))
    (:file "_package_WheelAngularVelocityPair" :depends-on ("_package"))
  ))