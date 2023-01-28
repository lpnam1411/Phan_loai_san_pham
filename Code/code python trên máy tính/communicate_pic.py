import serial
import time
import numpy as np
import cv2
import qrcode 
from sinh_qr import generate_qr,take_pic
from qr_code import scan
###################################
com = 6
port = 2
#_________________________
seri = serial.Serial('COM'+str(com),9600)
seri.timeout =1
while True:
    command_ = seri.readline().decode('ascii')
    if command_ == '':
        print("no control signal yet")
    elif command_ == 'check\n':
        # name_ = input("Name image = ")
        result = scan(port)
        # test = 'a'
        seri.write(result.encode('ascii'))
        print('messenger is :',result)
    else:
        print('messenger is :',command_)
    # i = input('on/off :').strip()
    # if i =='done':
    #     print('stop transmit')
    #     break
    # seri.write(i.encode())
    # time.sleep(0.5)
    # print(seri.readline().decode('ascii'))
seri.close()

    