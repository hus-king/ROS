#!/usr/bin/env python

import rospy
import cv2
from ros_opencv import ROS2OPENCV
from std_msgs.msg import String
import os
import sys

class TakePhoto(ROS2OPENCV):
    def __init__(self, node_name):
        super(TakePhoto, self).__init__(node_name)
        self.photo_dirname = "/home/bcsh/px4_ws/src/camera_com/photos/"
        self.video_dirname = "/home/bcsh/px4_ws/src/camera_com/videos/"

        self.video_name = None
        self.if_record = False

        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_name = self.video_dirname + str(rospy.Time.now()) + '.avi'

        self.custom_activity_sub = rospy.Subscriber("/gesture/command", String, self.custom_activity_callback)
        if (not os.path.isdir(self.photo_dirname)):
            os.makedirs(self.photo_dirname)
        if (not os.path.isdir(self.video_dirname)):
            os.makedirs(self.video_dirname)
    
    def process_image(self, frame):
        src = frame.copy()
        result = src.copy()
        self.result = result

        if self.if_record:
            self.out.write(self.result)
        return result
    
    def custom_activity_callback(self, msg):
        rospy.loginfo("Received Camera ACtivity")
        if msg.data == "Takephoto":
            photo_dirname = self.result
            photo_name = str(rospy.Time.now())
            cv2.imwrite(self.photo_dirname + photo_name + '.png', self.result, [int(cv2.IMWRITE_JPEG_QUALITY), 95] )
            rospy.loginfo("Takephoto")
        
        elif msg.data == "Startrecord":
            self.if_record = True
            self.video_name = self.video_dirname + str(rospy.Time.now()) + '.avi'
            self.out = cv2.VideoWriter(self.video_name, self.fourcc, 20.0, (640, 480))
            rospy.loginfo("Startrecord")
        
        elif msg.data == "Stoprecord":
            self.if_record = False
            rospy.loginfo("Stoprecord")
        
if __name__ == '__main__':
    try:
        node_name = "take_photo"
        TakePhoto(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down take_photo node."
cv2.destroyAllWindows()
