# -*- encoding: UTF-8 -*-
#CONTROL DE LA CABEZA CON RESPECTO DEL CENTROIDE DEL objeto

import sys
from naoqi import ALProxy
import time
import almath
import vision_sensor_deteccion_cam1

def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("LElbowYaw", 1.0)

    # Simple command for the HeadYaw joint at 10% max speed
    names            = "HeadYaw"
    angles           = -90*almath.TO_RAD
    fractionMaxSpeed = 0.1
    motionProxy.setAngles(names,angles,fractionMaxSpeed)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Head", 0.0)

if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python almotion_controllingjoints.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
