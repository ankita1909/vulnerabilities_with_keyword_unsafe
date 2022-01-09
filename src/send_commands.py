#!/usr/bin/python3

import serial
import time

serialPort = 'COM9'
serialBaud = 9600

commands = ["0xccddeeff\r","0x8899aabb","0x44556677","0x00112233", "0xd"]

ser = serial.Serial(serialPort, serialBaud)
ser.close()
ser.open()

while 1:
    ser.write("hello".encode('ascii'))

    #ser.write(commands[0].encode('utf-8'))
    #print(commands[0].encode('utf-8'))
    time.sleep(2)

    data = ser.read(100)
    #time.sleep(0.4)
    if data != b'\x00':
        print(data)
        print(data.decode("ascii"))

        #print("recieved: "+str([ "%x" % r for r in data]))


   # if data!=0xffffffff:
    #    print(data)
    #else:
     #   ser.write("Hello\n")
      #  ser.write(commands.__iter__)

ser.close()
