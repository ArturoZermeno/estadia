# -*- encoding: UTF-8 -*-
import cv2
import sys
from naoqi import ALProxy
import time
import almath
import math
def main(robotIP,pos):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)
    useSensors = False
    robotPosition     = almath.Pose2D(motionProxy.getRobotPosition(useSensors))
    print  robotPosition[2]

    cv2.waitKey(0)


    motionProxy.setStiffnesses("LElbowYaw", 1.0)

    # Simple command for the HeadYaw joint at 10% max speed
    names            = "HeadYaw"
    angles           = pos*almath.TO_RAD
    fractionMaxSpeed = 0.1
    motionProxy.setAngles(names,angles,fractionMaxSpeed)

    time.sleep(3.0)
    motionProxy.setStiffnesses("Head", 0.0)

def run(x):
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python almotion_controllingjoints.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]
    print x
    pos=math.sqrt((320-x)*(320-x))
    print pos



    if pos<0:
        pos=90
    else:
        pos=-90

    main(robotIp,pos)

if __name__ == "__main__":
    run(x)
