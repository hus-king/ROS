#!/usr/bin/env python


import rospy

from camera_processor.msg import PointDepth


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera_processor/depth/points", PointDepth, callback)

    rospy.spin()


if __name__ == '__main__':

    listener()


