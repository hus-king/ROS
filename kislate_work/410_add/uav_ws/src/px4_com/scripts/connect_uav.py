#!/usr/bin/env python
import time
from serial_helper import SerialHelper
import threading
import rospy
from std_msgs.msg import String


class UPUavControl():

    def __init__(self):
       
        self.ser = SerialHelper()
        self.ser.on_connected_changed(self.myserial_on_connected_changed)

       
        self._isConn = False


    def myserial_on_connected_changed(self, is_connected):
        if is_connected:
            print("Connected")
            self._isConn = True
            self.ser.connect()
        else:
            print("DisConnected")


    def write(self, data):
        self.ser.write(data, True)


    def generateCmd(self, device, len, cmd, data):
        buffer = [0] * (len + 6)
        buffer[0] = 0xFF
        buffer[1] = 0xFF
        buffer[2] = device & 0xFF
        buffer[3] = 0x07
        buffer[4] = cmd & 0xFF
        for i in range(len):
            buffer[5 + i] = data[i]

        check = 0
        for i in range(len + 3):
            check += buffer[i + 2]

        buffer[len + 5] = (~check) & 0xFF

        for i in range(len + 6):
            print(str(hex(buffer[i])))

        return buffer

    def setServoPosition(self, angel):
        data = [0] * 5
        data[0] = 0x2A
        data[1] = (angel >> 8) & 0x00FF
        data[2] = angel & 0x00FF
        data[3] = 0
        data[4] = 0
        buffer = self.generateCmd(0x01, 0x05, 0x03, data)
        self.write(buffer)

def servo_controller_callback(data):
    connect = UPUavControl()
    time.sleep(0.3)
    if(data.data == "1"):
        connect.setServoPosition(1700)
    if(data.data == "2"):
        connect.setServoPosition(1024)
    if(data.data == "3"):
        connect.setServoPosition(512)
        time.sleep(3)
        connect.setServoPosition(2048)
    


if __name__ == '__main__':
    rospy.init_node("servo_controller_node")
    rospy.Subscriber("servo_controller", String, servo_controller_callback)
    rospy.spin()









