import numpy as np
import math
#joints Definition
ShoulderOffsetY=98.0
ElbowOffsetY=15.0
UpperArmLength=105.0
ShoulderOffsetZ=100.0
LowerArmLength=57.75
HandOffsetX=55.95
HandOffsetZ=12.31
HipOffsetY=50.0
HipOffsetZ=85.0
ThighLength=100.0
TibiaLength=102.9
FootHeight=45.11
NeckOffsetZ=126.5
CameraBotomX=48.8
CameraBotomZ=23.81
CameraTopX=53.9
CameraTopZ=67.9

#Head Limits
HeadYawHigh=2.0857
HeadYawLow=-2.0857
HeadPitchHigh=0.5149
HeadPitchLow=-0.6720
#Left Hand limits
LShoulderPitchHigh=2.0857
LShoulderPitchLow=-2.0857
LShoulderRollHigh=1.3265
LShoulderRollLow=-0.3142
LElbowYawHigh=2.0875
LElbowYawLow=-2.0875
LElbowRollHigh=0.00001
LElbowRollLow=-1.5446
LWristYawHigh=1.8238
LWristYawLow=-1.8238

#Right Hand limits
RShoulderPitchHigh=2.0857
RShoulderPitchLow=-2.0857
RShoulderRollHigh=0.3142
RShoulderRollLow=-1.3265
RElbowYawHigh=2.0875
RElbowYawLow=-2.0875
RElbowRollHigh=1.5446
RElbowRollLow=0.00001
RWristYawHigh=1.8238
RWristYawLow=-1.8238
#Left Leg limits
LHipYawPitchHigh=0.7408
LHipYawPitchLow=-1.1453
LHipRollHigh=0.7904
LHipRollLow=-0.3794
LHipPitchHigh=0.4840
LHipPitchLow=-1.7739
LKneePitchHigh=2.1125
LKneePitchLow=-0.0923
LAnklePitchHigh=0.9227
LAnklePitchLow=-1.1895
LAnkleRollHigh=0.7690
LAnkleRollLow=-0.3978
#Left Right limits
RHipYawPitchHigh=0.7408
RHipYawPitchLow=-1.1453
RHipRollHigh=0.4147
RHipRollLow=-0.7383
RHipPitchHigh=0.4856
RHipPitchLow=-1.7723
RKneePitchHigh=2.1201
RKneePitchLow=-0.1030
RAnklePitchHigh=0.9320
RAnklePitchLow=-1.1864
RAnkleRollHigh=0.3886
RAnkleRollLow=-1.1864

#Masses defines
#Total mass
TotalMassH25=(5.182530+0.345)
#Torso
TorsoMass=1.04956
TorsoX=-4.13
TorsoY=0.09
TorsoZ=43.42

#Not provided by aldebaran
BatteryMass=0.345
BatteryX=-30.00
BatteryY=0.00
BatteryZ=39.00

#Head
HeadYawMass=0.06442
HeadYawX=-0.01
HeadYawY=0.14
HeadYawZ=-27.42

HeadPitchMass=0.60533
HeadPitchX=-1.12
HeadPitchY=	0.0
HeadPitchZ=52.58

#Right Hand
RShoulderPitchMass=0.06996
RShoulderPitchX=-1.65
RShoulderPitchY=26.63
RShoulderPitchZ=0.14

RShoulderRollMass=0.15794
RShoulderRollX=24.29
RShoulderRollY=-9.52
RShoulderRollZ=0.32

RElbowYawMass=0.06483
RElbowYawX=-27.44
RElbowYawY=	0.00
RElbowYawZ= 0.14

RElbowRollMass=	0.07778
RElbowRollX=25.52
RElbowRollY=-2.81
RElbowRollZ=0.9

RWristYawMass=0.18533
RWristYawX=LowerArmLength+34.34
RWristYawY=	-0.88
RWristYawZ=3.08

#Right Leg
RHipYawPitchMass=0.07118
RHipYawPitchX=-7.66
RHipYawPitchY=12.00
RHipYawPitchZ=27.16

RHipRollMass=0.13053
RHipRollX=-15.49
RHipRollY=-0.29
RHipRollZ=-5.16

RHipPitchMass=0.38976
RHipPitchX=1.39
RHipPitchY=-2.25
RHipPitchZ=-53.74

RKneePitchMass=0.29163
RKneePitchX=3.94
RKneePitchY=-2.21
RKneePitchZ=-49.38

RAnklePitchMass=0.13415
RAnklePitchX=0.45
RAnklePitchY=-0.3
RAnklePitchZ=6.84

RAnkleRollMass=	0.16171
RAnkleRollX=25.42
RAnkleRollY=-3.32
RAnkleRollZ=-32.39

def DH_Head(theta_1_head,theta_2_head):
    #Angulos de entrada
    #theta_1_head
    #theta_2_head
    #A translation matrix
    A_head=np.array([0,0,NeckOffsetZ])

    Base_head=np.array([A_head,A_head,A_head,A_head])
    HeadYaw=np.array([0,0,0,theta_1_head])
    HeadPitch=np.array([0,-math.pi/2,0,theta_2_head-(math.pi/2)])

    R_x=R_y=math.pi/2 #Rotation simple

    Rotation_head=np.array([[R_x,R_y],[R_x,R_y],[R_x,R_y],[R_x,R_y]])

    A_top_camera=np.array([CameraTopX,0,CameraTopZ])
    A_bottom_camera=np.array([CameraBotomX,0,CameraBotomZ])

    Top_camera=np.array([A_top_camera,A_top_camera,A_top_camera,A_top_camera])
    Bottom_camera=np.array([A_bottom_camera,A_bottom_camera,A_bottom_camera,A_bottom_camera])


    head_chain_dh=np.array([[Base_head],HeadYaw,HeadPitch,Rotation_head,Top_camera,Bottom_camera])



def DH_LArm(theta_1_larm,theta_2_larm,theta_3_larm,theta_4_larm):
    #Angulos de entrada
    #theta_1_larm
    #theta_2_larm
    #theta_3_larm
    #theta_4_larm
    #A translation matrix
    A_base=np.array([0,ShoulderOffsetY+ElbowOffsetY,ShoulderOffsetZ])

    Base_larm=np.array([A_base,A_base,A_base,A_base])
    LShoulderPitch=np.array([0,-math.pi/2,0,theta_1_larm])
    LShoulderRoll=np.array([0,math.pi/2,0,theta_2_larm-(math.pi/2)])
    LElbowYaw=np.array([0,math.pi/2,UpperArmLength,-theta_3_larm])
    LElbowRoll=np.array([0,math.pi/2,0,theta_4_larm])

    R_z=math.pi/2 #Rotation simple

    Rotation_larm=np.array([R_z,R_z,R_z,R_z])

    A_end_effector=np.array([HandOffsetX+LowerArmLength,0,0])


    End_Effector_LArm=np.array([A_end_effector,A_end_effector,A_end_effector,A_end_effector])



    larm_chain_dh=np.array([[Base_larm],LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll,Rotation_larm,End_Effector_LArm])


    print larm_chain_dh[6][0][0]

def DH_RArm(theta_1_rarm,theta_2_rarm,theta_3_rarm,theta_4_rarm):
    #Angulos de entrada
    #theta_1_larm
    #theta_2_larm
    #theta_3_larm
    #theta_4_larm
    #A translation matrix
    A_base=np.array([0,-ShoulderOffsetY-ElbowOffsetY,ShoulderOffsetZ])

    Base_rarm=np.array([A_base,A_base,A_base,A_base])
    RShoulderPitch=np.array([0,-math.pi/2,0,theta_1_rarm])
    RShoulderRoll=np.array([0,math.pi/2,0,theta_2_rarm+(math.pi/2)])
    RElbowYaw=np.array([0,math.pi/2,-UpperArmLength,theta_3_rarm])
    RElbowRoll=np.array([0,math.pi/2,0,theta_4_rarm])

    R_z=math.pi/2 #Rotation simple

    Rotation_rarm=np.array([R_z,R_z,R_z,R_z])

    A_end_effector=np.array([-HandOffsetX-LowerArmLength,0,0])


    End_Effector_RArm=np.array([A_end_effector,A_end_effector,A_end_effector,A_end_effector])

    Rotation_Fix=np.array([-math.pi,-math.pi,-math.pi,-math.pi])

    rarm_chain_dh=np.array([[Base_rarm],RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll,Rotation_rarm,End_Effector_RArm,Rotation_Fix])


    print rarm_chain_dh[7][0]

if __name__ == '__main__':
    print "================ DH PARAMETERS ================"
    theta_1_head=1
    theta_2_head=1

    theta_1_larm=1
    theta_2_larm=1
    theta_3_larm=1
    theta_4_larm=1

    theta_1_rarm=1
    theta_2_rarm=1
    theta_3_rarm=1
    theta_4_rarm=1

    DH_Head(theta_1_head,theta_2_head)
    DH_LArm(theta_1_larm,theta_2_larm,theta_3_larm,theta_4_larm)
    DH_RArm(theta_1_rarm,theta_2_rarm,theta_3_rarm,theta_4_rarm)
