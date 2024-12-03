#!/usr/bin/env python

import rospy
import cv2
import time

from pyzbar import pyzbar
from std_msgs.msg import String

cap = cv2.VideoCapture(0)
ret = cap.set(3, 640)  
ret = cap.set(4, 480)  

class QrCodeScan():
    def __init__(self):

        self.scan_content = ""
        self.qr_code_scan_pub = rospy.Publisher('qr_code_scan_content', String, queue_size=10)
        self.start_video()

    def start_video(self):
        while cap.isOpened():
            ret, frame = cap.read()
            result = frame.copy()

            
            barcodes = pyzbar.decode(result)
            
            for barcode in barcodes:
                
                (x, y, w, h) = barcode.rect
                cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 5)
                
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                
                text = "{}".format(barcodeData, barcodeType)
                self.scan_content = barcodeData
                
                cv2.putText(result, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 0), 2)
                self.qr_code_scan_pub.publish(self.scan_content)
                

            
            cv2.imshow("result",result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node_name = "qr_code_scan"
    rospy.init_node(node_name,log_level=rospy.DEBUG)
    # rospy.spin()
    qr_code_scan = QrCodeScan()











