import GlobalHeader
from SerialManager import SerialProcess
import time
import random
from PIL import Image
import numpy as np
import io
time.sleep(3)
ser2 = SerialProcess()
ser2.serial.port = "COM1"
ser2.serial.setBaud(115200)

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
rotx = 0
roty = 0
rotz = 0


def turnToStringList(listOfVars):
    strList = ""
    for e in listOfVars:
        strList = strList + "," + str(e)

    return strList 


def level(rotx,roty,rotz):
    if(abs(rotx+roty+rotz) > 0.01):
        rotx = rotx - rotx*0.5
        roty = roty - roty*0.5
        rotz = rotz - rotz*0.5


    return rotx,roty,rotz   
    self.watchdog.sleep()

def generateImage(path):
    
    tempImg = Image.open(path)
    matImg = np.asarray(tempImg)
    x,y,colors = matImg.shape
    imgVect = matImg.ravel()

    img = imgVect

    return tempImg,x,y,colors

flightState = 4
time.sleep(3)
#ser2.writeData(bytes("\r\n",'utf-8'))
for i in range(10000):
    
    
    if(xz < 30 and flightState ==0):
        az = 2
    elif(flightState==0):
        az = -9.8
        flightState = 1
    

    if(vz <= 0 and flightState ==1):
        flightState = 2
    
    if(xz<=1 and missionTime >1 and flightState >= 1 and flightState < 5):
            flightState = 3
            xz = 0

    if((rotx+roty+rotz) <= 0.1 and flightState ==3):
            flightState = 4      

    if(flightState < 3):
        ax = ax+ (random.random()*2-1)
        ay = ay+ (random.random()*2-1)
        az = az

        rotx = rotx + (random.random()*10-5)
        roty = roty + (random.random()*10-5)
        rotz = rotz + (random.random()*10-5)

        #rotx = 0
        #roty = 0
        #rotz = 45

        ## Integrate things
        vx = integrate(ax,vx,timeStep)
        vy = integrate(ay,vy,timeStep)
        vz = integrate(az,vz,timeStep)

        xx = integrate(vx,xx,timeStep)
        xy = integrate(vy,xy,timeStep)
        xz = integrate(vz,xz,timeStep)

    if(flightState == 3):
        [rotx,roty,rotz] = level(rotx,roty,rotz)

        ax = 0
        ay = 0
        az = 0
        vx = 0
        vy = 0
        vz = 0


    #Transmit Image
    if(flightState == 4):
        ##
        for i in range(3):
            img,x,y,colors = generateImage("Test0.jpg")
            listOfVars = [",Image"]
            strList = turnToStringList(listOfVars)
            ser2.writeData(bytes(strList,'utf-8'))
            output = io.BytesIO()
            img.save(output,format='JPEG')
            hex_data = output.getvalue()
            
            count = 0
            #time.sleep(5)
            for i in range(len(hex_data)):
                if(i%10000 == 0):
                    if(i +10000>=len(hex_data)):
                        ser2.writeData(hex_data[i:])
                    else:
                        ser2.writeData(hex_data[i:i+10000])
                    #time.sleep(0.001)
            
            #time.sleep(0.1)
            #for i in img:
            #    temp = "," + str(i)
            #    ser2.writeData(bytes(temp,'utf-8'))
            #    time.sleep(0.01)
            temp2 = ",Image End"
            ser2.writeData(bytes(temp2,'utf-8'))
            #3

            
            time.sleep(5)
            img,x,y,colors = generateImage("Test1.jpg")
            listOfVars = [",Image"]
            strList = turnToStringList(listOfVars)
            ser2.writeData(bytes(strList,'utf-8'))
            output = io.BytesIO()
            img.save(output,format='JPEG')
            hex_data = output.getvalue()
            
            count = 0
            #time.sleep(5)
            for i in range(len(hex_data)):
                if(i%10000 == 0):
                    if(i +5000>=len(hex_data)):
                        ser2.writeData(hex_data[i:])
                    else:
                        ser2.writeData(hex_data[i:i+10000])
                    #time.sleep(0.001)
            
            time.sleep(0.1)
            #for i in img:
            #    temp = "," + str(i)
            #    ser2.writeData(bytes(temp,'utf-8'))
            #    time.sleep(0.01)
            temp2 = ",Image End"
            ser2.writeData(bytes(temp2,'utf-8'))

            
            time.sleep(5)
            img,x,y,colors = generateImage("Test2.jpg")
            listOfVars = [",Image"]
            strList = turnToStringList(listOfVars)
            ser2.writeData(bytes(strList,'utf-8'))
            output = io.BytesIO()
            img.save(output,format='JPEG')
            hex_data = output.getvalue()
            
            count = 0
            #time.sleep(5)
            for i in range(len(hex_data)):
                if(i%10000 == 0):
                    if(i +10000>=len(hex_data)):
                        ser2.writeData(hex_data[i:])
                    else:
                        ser2.writeData(hex_data[i:i+10000])
                    #time.sleep(0.001)
            
            #time.sleep(0.1)
            #for i in img:
            #    temp = "," + str(i)
            #    ser2.writeData(bytes(temp,'utf-8'))
            #    time.sleep(0.01)
            temp2 = ",Image End"
            ser2.writeData(bytes(temp2,'utf-8'))

            flightState = 5
    else:

        listOfVars = [teamName,i,missionTime,flightState,xz,ax,ay,az,rotx,roty,rotz,teamName+" End"]
        strList = turnToStringList(listOfVars)
        ser2.writeData(bytes(strList,'utf-8'))

    time.sleep(timeStep)
    missionTime = missionTime + timeStep
