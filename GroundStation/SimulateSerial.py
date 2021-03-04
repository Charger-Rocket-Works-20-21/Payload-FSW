import GlobalHeader
from SerialManager import SerialProcess
import time
import random

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


def turnToStringList(listOfVars):
    strList = ""
    for e in listOfVars:
        strList = strList + "," + str(e)

    return strList
    


for i in range(500):
    
    if(xz < 1000):
        az = 100
    else:
        az = -9.8
        flightState = 1
    
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


    listOfVars = [teamName,missionTime,flightState,xz,ax,ay,az]
    strList = turnToStringList(listOfVars)
    ser2.writeData(bytes(strList,'utf-8'))

    time.sleep(timeStep)
    missionTime = missionTime + timeStep
