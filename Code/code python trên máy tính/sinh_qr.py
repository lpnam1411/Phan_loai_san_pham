#Import Library
import cv2
from cv2 import waitKey
from matplotlib.pyplot import flag
import qrcode
import time
#Generate QR Code
def take_pic(port):
    cam = cv2.VideoCapture(port, cv2.CAP_DSHOW)
    cam.set(3,640)
    cam.set(4,480)
    for i in range(10):
        flag,img_test = cam.read()
    flag,img = cam.read()
    cv2.destroyAllWindows()
    cv2.VideoCapture(port).release()
    return img

def generate_qr(name,content):
    img=qrcode.make("{}".format(content))
    img.save('{}.png'.format(name))

generate_qr("c_img","c")
