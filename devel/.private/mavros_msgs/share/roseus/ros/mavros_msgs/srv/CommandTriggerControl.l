;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::CommandTriggerControl)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'CommandTriggerControl (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::COMMANDTRIGGERCONTROL")
  (make-package "MAVROS_MSGS::COMMANDTRIGGERCONTROL"))
(unless (find-package "MAVROS_MSGS::COMMANDTRIGGERCONTROLREQUEST")
  (make-package "MAVROS_MSGS::COMMANDTRIGGERCONTROLREQUEST"))
(unless (find-package "MAVROS_MSGS::COMMANDTRIGGERCONTROLRESPONSE")
  (make-package "MAVROS_MSGS::COMMANDTRIGGERCONTROLRESPONSE"))

(in-package "ROS")





(defclass mavros_msgs::CommandTriggerControlRequest
  :super ros::object
  :slots (_trigger_enable _cycle_time ))

(defmethod mavros_msgs::CommandTriggerControlRequest
  (:init
   (&key
    ((:trigger_enable __trigger_enable) nil)
    ((:cycle_time __cycle_time) 0.0)
    )
   (send-super :init)
   (setq _trigger_enable __trigger_enable)
   (setq _cycle_time (float __cycle_time))
   self)
  (:trigger_enable
   (&optional __trigger_enable)
   (if __trigger_enable (setq _trigger_enable __trigger_enable)) _trigger_enable)
  (:cycle_time
   (&optional __cycle_time)
   (if __cycle_time (setq _cycle_time __cycle_time)) _cycle_time)
  (:serialization-length
   ()
   (+
    ;; bool _trigger_enable
    1
    ;; float32 _cycle_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _trigger_enable
       (if _trigger_enable (write-byte -1 s) (write-byte 0 s))
     ;; float32 _cycle_time
       (sys::poke _cycle_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _trigger_enable
     (setq _trigger_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _cycle_time
     (setq _cycle_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mavros_msgs::CommandTriggerControlResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::CommandTriggerControlResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _result (round __result))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; uint8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mavros_msgs::CommandTriggerControl
  :super ros::object
  :slots ())

(setf (get mavros_msgs::CommandTriggerControl :md5sum-) "7036c5f362c09a051915015871ce633d")
(setf (get mavros_msgs::CommandTriggerControl :datatype-) "mavros_msgs/CommandTriggerControl")
(setf (get mavros_msgs::CommandTriggerControl :request) mavros_msgs::CommandTriggerControlRequest)
(setf (get mavros_msgs::CommandTriggerControl :response) mavros_msgs::CommandTriggerControlResponse)

(defmethod mavros_msgs::CommandTriggerControlRequest
  (:response () (instance mavros_msgs::CommandTriggerControlResponse :init)))

(setf (get mavros_msgs::CommandTriggerControlRequest :md5sum-) "7036c5f362c09a051915015871ce633d")
(setf (get mavros_msgs::CommandTriggerControlRequest :datatype-) "mavros_msgs/CommandTriggerControlRequest")
(setf (get mavros_msgs::CommandTriggerControlRequest :definition-)
      "

bool    trigger_enable
float32 cycle_time
---
bool success
uint8 result

")

(setf (get mavros_msgs::CommandTriggerControlResponse :md5sum-) "7036c5f362c09a051915015871ce633d")
(setf (get mavros_msgs::CommandTriggerControlResponse :datatype-) "mavros_msgs/CommandTriggerControlResponse")
(setf (get mavros_msgs::CommandTriggerControlResponse :definition-)
      "

bool    trigger_enable
float32 cycle_time
---
bool success
uint8 result

")



(provide :mavros_msgs/CommandTriggerControl "7036c5f362c09a051915015871ce633d")


