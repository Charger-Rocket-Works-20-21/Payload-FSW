import GlobalHeader
from SerialManager import SerialProcess
import time
import random
from PIL import Image
import numpy as np
ser2 = SerialProcess()
ser2.serial.port = "COM2"

ser2.connectDevice()



teamName = "UAH Charger RocketWorks"
missionTime = 0
timeStep = 0.5
flightState = 0

def integrate(dx,x0, timestep):
    return dx * timestep + x0

ax = 0
ay = 0
az = 0
vx = 0
vy = 0
vz = 0
xx = 0
xy = 0
xz = 0
rotx = random.random()*90.0
roty = random.random()*90.0
rotz = random.random()*90.0


def turnToStringList(listOfVars):
    strList = ""
    for e in listOfVars:
        strList = strList + "," + str(e)

    return strList


def level(rotx,roty,rotz):
    if((rotx+roty+rotz) > 0.1):
        rotx = rotx - rotx*0.5
        roty = roty - roty*0.5
        rotz = rotz - rotz*0.5


    return rotx,roty,rotz   


def generateImage():
    
    tempImg = Image.open("TestImg.png")
    matImg = np.asarray(tempImg)
    x,y,colors = matImg.shape
    imgVect = matImg.ravel()

    img = imgVect

    return img,x,y,colors



for i in range(500):
    
    if(xz < 5 and flightState ==0):
        az = 10
    elif(flightState==0):
        az = -9.8
        flightState = 1
    

    if(vz <= 0 and flightState ==1):
        flightState = 2
    
    if(xz<=1 and missionTime >5 and flightState == 2):
            flightState = 3
            xz = 0

    if((rotx+roty+rotz) <= 0.1 and flightState ==3):
            flightState = 4      

    if(flightState < 3):
        ax = ax+random.random()
        ay = ay+random.random()
        az = az+random.random()

        ## Integrate things
        vx = integrate(ax,vx,timeStep)
        vy = integrate(ay,vy,timeStep)
        vz = integrate(az,vz,timeStep)

        xx = integrate(vx,xx,timeStep)
        xy = integrate(vy,xy,timeStep)
        xz = integrate(vz,xz,timeStep)

    if(flightState == 3):
        [rotx,roty,rotz] = level(rotx,roty,rotz)


    #Transmit Image
    if(flightState == 4):
        img,x,y,colors = generateImage()
        listOfVars = ["Image",x,y,colors]
        strList = turnToStringList(listOfVars)
        ser2.writeData(bytes(strList,'utf-8'))
        for i in img:
            temp = "," + str(i)
            ser2.writeData(bytes(temp,'utf-8'))
        temp2 = ",Image End"
        ser2.writeData(bytes(temp2,'utf-8'))
        flightState = 5
    else:

        listOfVars = [teamName,missionTime,flightState,xz,ax,ay,az,rotx,roty,rotz,teamName+" End"]
        strList = turnToStringList(listOfVars)
        ser2.writeData(bytes(strList,'utf-8'))

    time.sleep(timeStep)
    missionTime = missionTime + timeStep
