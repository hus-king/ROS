#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import cv2
import numpy as np
from px4_command.msg import command
import time


def face_rec(cmd):
    command_id = 0
    face_cascade = cv2.CascadeClassifier('/home/bcsh/uav_ws/src/px4_com/cascades/haarcascade_frontalface_default.xml')
    pipeline='v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,format=I420,framerate=30/1  !  videoconvert ! video/x-raw,width=640,height=480,format=BGR ! appsink'
    cap=cv2.VideoCapture(0)
    # cap = cv2.VideoCapture("nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    while cap.isOpened():
        ret, frame = cap.read()
        result = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 2)
            if faces is not None:
                if len(faces) == 1:
                    (fx, fy, fw, fh) = faces[0]
                    target_face_x = fx + fw/2
                    print(target_face_x)
                    offset_x = target_face_x - 320
                    offset_x = -offset_x
                    print(offset_x)

                    if offset_x < -10:
                        rospy.loginfo("move right")
                        px4_cmd = command()
                        px4_cmd.command = 0
                        px4_cmd.comid = command_id
                        command_id += 1
                        px4_cmd.sub_mode = 2

                        px4_cmd.vel_sp[0] = 0
                        px4_cmd.vel_sp[1] = -0.1

                        px4_cmd.pos_sp[2] = 0.5
                        px4_cmd.yaw_sp = 0
                        cmd.publish(px4_cmd)

                    if offset_x > 10:
                        rospy.loginfo("move left")
                        px4_cmd = command()
                        px4_cmd.command = 0
                        px4_cmd.comid = command_id
                        command_id += 1
                        px4_cmd.sub_mode = 2

                        px4_cmd.vel_sp[0] = 0
                        px4_cmd.vel_sp[1] = 0.1

                        px4_cmd.pos_sp[2] = 0.5
                        px4_cmd.yaw_sp = 0
                        cmd.publish(px4_cmd)

        cv2.namedWindow("recognize_face", 0)
        cv2.resizeWindow("recognize_face", 640, 480)
        cv2.imshow("recognize_face", result)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    node_name = "face_detector"
    rospy.init_node(node_name,log_level=rospy.DEBUG)
    cmd_now = 0

    cmd = rospy.Publisher('/px4/command', command, queue_size=10)
    arm_cmd = input("please input cmd 1 is Arm other is exit ?")
    rospy.loginfo(arm_cmd)
    if arm_cmd == 1:
        px4_cmd = command()
        px4_cmd.command = 5
        cmd.publish(px4_cmd)
    else:
        sys.exit(0)

    takeoff_cmd = input("please input cmd 1 is TakeOff other is exit ?")

    if takeoff_cmd == 1:
        px4_cmd = command()
        px4_cmd.command = 3
        cmd.publish(px4_cmd)
    else:
        sys.exit(0)

    rec_cmd = input("please input cmd 1 is rec_cmd other is exit ?")

    if rec_cmd == 1:
        face_rec(cmd)
    else:
        sys.exit(0)

    land_cmd = input("please input cmd 1 is Land other is exit ?")

    if land_cmd == 1:
	px4_cmd = command()
        px4_cmd.command = 4
	while True:
            cmd.publish(px4_cmd)
    else:
        sys.exit(0)
    rospy.spin()











