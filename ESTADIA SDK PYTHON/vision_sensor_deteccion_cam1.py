"""
Created on Thu Jul  9 15:30:54 2015

@author: Pierre Jacquot
"""
import cv2
import numpy as np

import math
import almath
from naoqi import ALProxy

import array
from scripts import vrep
from nao_motion import almotion_controlling_joints
import time,sys
import matplotlib.pyplot as plt
from PIL import Image as I
import array

def streamVisionSensor(visionSensorName,clientID,pause=0.0001):
    #Get the handle of the vision sensor
    res1,visionSensorHandle=vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
    #Allow the display to be refreshed
    plt.ion()
    #Initialiazation of the figure
    time.sleep(0.5)
    res,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
    im = I.new("RGB", (resolution[0], resolution[1]), "white")
    #Give a title to the figure
    fig = plt.figure(1)
    fig.canvas.set_window_title(visionSensorName)
    #inverse the picture
    plotimg = plt.imshow(im,origin='lower')
    #Let some time to Vrep in order to let him send the first image, otherwise the loop will start with an empty image and will crash
    time.sleep(1)
    print (resolution)
    while (vrep.simxGetConnectionId(clientID)!=-1):
        #Get the image of the vision sensor
        res,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
        #Transform the image so it can be displayed using pyplot
        image_byte_array = array.array('b',image)
        im = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        #Update the image
        plotimg.set_data(im)
        #Refresh the display
        plt.draw()
        #The mandatory pause ! (or it'll not work)
        plt.pause(pause)
        img = np.array(image, dtype = np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img = np.rot90(img,2)
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


        #Convertir img a hsv y detectar colores
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        verde_bajos = np.array([49,50,50], dtype=np.uint8)
        verde_altos = np.array([80, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, verde_bajos, verde_altos) #Crear mascara

        #Limpiar mascara y buscar centro del objeto verde
        moments = cv2.moments(mask)
        area = moments['m00']
        cv2.circle(img,(resolution[0]/2,resolution[1]/2),2 ,(0,0,255), 2)

        if(area > 200):
            x = int(moments['m10']/moments['m00'])
            y = int(moments['m01']/moments['m00'])
            cv2.rectangle(img, (x, y), (x+2, y+2),(0,0,255), 2)

            #Descomentar para printear la posicion del centro
            #print(x,y)

        print math.sqrt((x-(resolution[0]/2))**2)+((y-(resolution[1]/2))**2)

        #almotion_controlling_joints.run(x)


        #Mostrar frame y salir con "ESC"
        cv2.imshow('Image', img)
        cv2.imshow('Mask', mask)
        tecla = cv2.waitKey(5) & 0xFF
        if tecla == 27:
            break

    print 'End of Simulation'

def getVisionSensor(visionSensorName,clientID):
    #Get the handle of the vision sensor
    res1,visionSensorHandle=vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
    time.sleep(1)
    while (vrep.simxGetConnectionId(clientID)!=-1):
        #Get the image of the vision sensor
        res,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
        print resolution
    print 'End of Simulation'

def run():
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.3',19998,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
            #Get and display the pictures from the camera
        streamVisionSensor('NAO_vision1',clientID,0.0001)
            #Only get the image
            #getVisionSensor('NAO_vision1',clientID)

    else:
        print 'Connection non successful'
        sys.exit('Could not connect')


if __name__ == '__main__':
    print "================ Sensor Vision ================"
    run()
#if __name__ == '__main__':
#    vrep.simxFinish(-1)
#    clientID=vrep.simxStart('127.0.0.3',19998,True,True,5000,5)
#    if clientID!=-1:
#        print 'Connected to remote API server'
        #Get and display the pictures from the camera
#        streamVisionSensor('NAO_vision1',clientID,0.0001)
        #Only get the image
        #getVisionSensor('NAO_vision1',clientID)

#    else:
#        print 'Connection non successful'
#        sys.exit('Could not connect')
