
(cl:in-package :asdf)

(defsystem "ros_system-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "SubstractTwoInts" :depends-on ("_package_SubstractTwoInts"))
    (:file "_package_SubstractTwoInts" :depends-on ("_package"))
  ))