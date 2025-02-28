;; Auto-generated. Do not edit!


(when (boundp 'tutorial_vision::CircleInfo)
  (if (not (find-package "TUTORIAL_VISION"))
    (make-package "TUTORIAL_VISION"))
  (shadow 'CircleInfo (find-package "TUTORIAL_VISION")))
(unless (find-package "TUTORIAL_VISION::CIRCLEINFO")
  (make-package "TUTORIAL_VISION::CIRCLEINFO"))

(in-package "ROS")
;;//! \htmlinclude CircleInfo.msg.html


(defclass tutorial_vision::CircleInfo
  :super ros::object
  :slots (_center_x _center_y _radius ))

(defmethod tutorial_vision::CircleInfo
  (:init
   (&key
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _radius (float __radius))
   self)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; float64 _center_x
    8
    ;; float64 _center_y
    8
    ;; float64 _radius
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _center_x
     (setq _center_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _center_y
     (setq _center_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get tutorial_vision::CircleInfo :md5sum-) "28a09b8c5ded5c4af6b7db31b0bee222")
(setf (get tutorial_vision::CircleInfo :datatype-) "tutorial_vision/CircleInfo")
(setf (get tutorial_vision::CircleInfo :definition-)
      "float64 center_x
float64 center_y
float64 radius

")



(provide :tutorial_vision/CircleInfo "28a09b8c5ded5c4af6b7db31b0bee222")


