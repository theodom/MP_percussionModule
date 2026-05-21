#!/usr/bin/python3
#   Run this script to reset the Arduino Nano Every
#   eg. sudo python3 mcu_reset.py
#
# Note: You must install pyserial first; i.e. sudo pip3 pyserial

import serial
import os, sys
# total arguments
n = len(sys.argv)
if n >= 2:
    port = sys.argv[1]
else:
    port = '/dev/ttyACM2'

print("Port: {}".format(port))

#re-enable hupcl temporarily (necessary for arduino reset using serial port)
os.system('sudo /bin/stty -F {} hupcl'.format(port))

try:
    #perform Arduino Nano Every reset "handshake"
    ser = serial.Serial()
    ser.baudrate = 1200
    ser.port = port
    ser.open()
    ser.close()

except serial.SerialException:
    print("Error: serial.SerialException")
    exit()