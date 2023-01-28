#Import Library
import cv2
from cv2 import waitKey
from numpy import true_divide
import qrcode
import time

def scan(port):
    cam = cv2.VideoCapture(port, cv2.CAP_DSHOW)
    cam.set(3,640)
    cam.set(4,480)
    val = ''
    for i in range(20):
        suc,img = cam.read()
    while val == '':
        suc,img = cam.read()
        cv2.imwrite('work_on.jpg',img)
        read_qr = cv2.imread('work_on.jpg')
        det = cv2.QRCodeDetector()
        val,pts,st_code = det.detectAndDecode(read_qr)
        cv2.destroyAllWindows()
        time.sleep(0.1)
    cv2.destroyAllWindows()
    cv2.VideoCapture(port).release()
    return val
            

# done = False
# while True:
#     check = input('signal = ')
#     if check == 'scan' :
#         cam = cv2.VideoCapture(port)
#         cam.set(3,640)
#         cam.set(4,480)
#         val = ''
#         flag = 0
#         while val == '':
#             suc,img = cam.read()
#             cv2.imwrite('work_on.jpg',img)
#             read_qr = cv2.imread('work_on.jpg')
#             det = cv2.QRCodeDetector()
#             val,pts,st_code = det.detectAndDecode(read_qr)
#             if val == '' and flag ==0:
#                 flag = 1
#                 print("Have not read QR yet")
#             elif val != '':
#                 print("the messenger is : ",val)
#                 cv2.imshow('Result',img)
#                 waitKey(1000)
#                 done = True
#         if done == True :
#             cv2.destroyAllWindows()
#             cv2.VideoCapture(port).release()
#             done = False
#     else:
#         pass
   


            
# err,day,mon,year, hours, minute = val[0:3],int(val[4:6]),int(val[7:9]),int(val[10:14]),int(val[15:17]),int(val[18:20])
# print('This product has error is "{}"\nDetail information\nDate : {}/{}/{}\nTime : {}:{}'.format(err,day,mon,year,hours,minute))