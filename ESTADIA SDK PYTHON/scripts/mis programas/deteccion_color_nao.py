import urllib
import cv2
import numpy as np
import time
from naoqi import ALProxy


url='http://192.168.137.89:8080/shot.jpg'
kernel=np.ones((5,5),np.uint8)
tts = ALProxy("ALTextToSpeech", "127.0.0.1", 9559)

while(1):
    # Use urllib to get the image from the IP camera
    imgResp = urllib.urlopen(url)
    # Numpy to convert into a array
    imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
    # Finally decode the array to OpenCV usable format ;)
    frame = cv2.imdecode(imgNp,-1)

    #Azul
    lower_blue = np.array([83,0,0])
    upper_blue = np.array([255,0,0])

    #Rojo
    lower_red = np.array([0,0,94])
    upper_red = np.array([0,0,255])

    #verde
    lower_green = np.array([0,47,0])
    upper_green = np.array([0,255,0])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(frame, lower_blue, upper_blue)
    mask2 = cv2.inRange(frame, lower_red, upper_red)
    mask3 = cv2.inRange(frame, lower_green, upper_green)

    if np.sum(mask)!= 0:
        tts.say("Ay algo azul")
        time.sleep(2)

    if np.sum(mask2)!= 0:
        tts.say("Ay algo rojo")
        time.sleep(2)

    if np.sum(mask3)!= 0:
        tts.say("Ay algo verde")
        time.sleep(2)




    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
