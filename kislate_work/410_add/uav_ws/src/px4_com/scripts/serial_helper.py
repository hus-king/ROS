# -*- coding: utf-8 -*-
import sys
import time
import serial
import logging
import binascii
import platform
import threading

if platform.system() == "Windows":
    from  serial.tools import list_ports
else:
    import glob, os, re

class SerialHelper(object):
    def __init__(self, Port="/dev/ttyUSB0", BaudRate="115200", ByteSize="8", Parity="N", Stopbits="1"):
        '''
        初始化一些参数
        '''
        self.port = Port
        self.baudrate = BaudRate
        self.bytesize = ByteSize
        self.parity = Parity
        self.stopbits = Stopbits
        self.threshold_value = 1
        self.receive_data = ""

        self._serial = None
        self._is_connected = False

    def connect(self, timeout=2):
        '''
        连接设备
        '''
        self._serial = serial.Serial()
        self._serial.port = self.port
        self._serial.baudrate = self.baudrate
        self._serial.bytesize = int(self.bytesize)
        self._serial.parity = self.parity
        self._serial.stopbits = int(self.stopbits)
        self._serial.timeout = timeout

        try:
            self._serial.open()
            if self._serial.isOpen():
                self._is_connected = True
        except Exception as e:
            self._is_connected = False
            logging.error(e)

    def disconnect(self):
        '''
        断开连接
        '''
        if self._serial:
            self._serial.close()

    def write(self, data, isHex=False):
        '''
        发送数据给串口设备
        '''
        if self._is_connected:
            self._serial.write(data)
        else:
            print("port no open")
#             if isHex:
#                 data = binascii.unhexlify(data)
#             self._serial.write(bytes(data))


    def on_connected_changed(self, func):
        '''
        set serial connected status change callback
        '''
        tConnected = threading.Thread(target=self._on_connected_changed, args=(func, ))
        tConnected.setDaemon(True)
        tConnected.start()

    def _on_connected_changed(self, func):
        '''
        set serial connected status change callback
        '''
        self._is_connected_temp = False
        while True:
            if platform.system() == "Windows":
                for com in list_ports.comports():
                    if com[0] == self.port:
                        self._is_connected = True
                        break
            elif platform.system() == "Linux":
                if self.port in self.find_usb_tty():
                    self._is_connected = True

            if self._is_connected_temp != self._is_connected:
                func(self._is_connected)
            self._is_connected_temp = self._is_connected
            time.sleep(0.5)

    def on_data_received(self, func):
        '''
        set serial data recieved callback
        '''
        tDataReceived = threading.Thread(target=self._on_data_received, args=(func, ))
        tDataReceived.setDaemon(True)
        tDataReceived.start()
    
    def _on_data_received(self, func):
        '''
        set serial data recieved callback
        '''
        while True:  
            if self._is_connected:
                
                try:
                    number = self._serial.inWaiting()
                    if number > 0:
                        data = self._serial.read(number)
                        if data:
#                             height = ((data[5] & 0xFF) | ((data[6] & 0xFF) << 8) | ((data[7] & 0xFF) << 16) | ((data[8] & 0xFF) << 24))
                            func(data)
                except Exception as e:
                    self._is_connected = False
                    self._serial = None
                    break
                
                
    def find_usb_tty(self, vendor_id=None, product_id=None):
        '''
        查找Linux下的串口设备
        '''
        tty_devs = list()
        for dn in glob.glob('/sys/bus/usb/devices/*') :
            try:
                vid = int(open(os.path.join(dn, "idVendor" )).read().strip(), 16)
                pid = int(open(os.path.join(dn, "idProduct")).read().strip(), 16)
                if  ((vendor_id is None) or (vid == vendor_id)) and ((product_id is None) or (pid == product_id)) :
                    dns = glob.glob(os.path.join(dn, os.path.basename(dn) + "*"))
                    for sdn in dns :
                        for fn in glob.glob(os.path.join(sdn, "*")) :
                            if  re.search(r"\/ttyUSB[0-9]+$", fn) :
                                tty_devs.append(os.path.join("/dev", os.path.basename(fn)))
            except Exception as ex:
                pass
        return tty_devs


if __name__ == '__main__':
    myserial = testHelper()

    
