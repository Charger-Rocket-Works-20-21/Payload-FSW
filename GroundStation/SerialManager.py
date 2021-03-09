from GlobalHeader import *

import serial.tools.list_ports
import serial
from serial import *
from PIL import Image
import numpy as np
portlist = [comport.name + ' - ' + comport.description for comport in serial.tools.list_ports.comports()]

#print('Currently Open Ports:')
for port in portlist: print(port)



class MySerial(QObject):
    isLogLocal = isLogGlobal
    baudRate = 0
    settings = QSettings()
    parity = 0
    dataBits = 0
    stopBits = 0
    flowControl = 0
    portList = []
    baudRateList = []
    serialPort = None
    port = "COM1"
    
    def __init__(self):
        super(MySerial,self).__init__()
        LOG("Serial Device Initialized")


    def openSerialPort(self):
        self.serialPort = QSerialPort(self.port)
        return self.serialPort





class SerialProcess(QObject):
    ## Instance Vars
    isLogLocal = False

    dataBuffer = QByteArray()
    device = None
    serial = None
    watchdog = None
    receivedBytes = 0
    numImagesReceived = 0
    frameTimer = None
    lockWhile = False
    lockRead = False
    ########################################################
    

    ## Signals
    tx                          = pyqtSignal()
    rx                          = pyqtSignal()
    deviceChanged               = pyqtSignal()
    connectedChanged            = pyqtSignal()
    watchdogTriggered           = pyqtSignal()
    dataSourceChanged           = pyqtSignal()
    writeEnabledChanged         = pyqtSignal()
    receivedBytesChanged        = pyqtSignal()
    maxBufferSizeChanged        = pyqtSignal()
    startSequenceChanged        = pyqtSignal()
    finishSequenceChanged       = pyqtSignal()
    watchdogIntervalChanged     = pyqtSignal()
    frameValidationRegexChanged = pyqtSignal()
    dataReceived                = pyqtSignal(['QByteArray'])
    frameReceived               = pyqtSignal(['QString'])
    imageReceived               = pyqtSignal(['QByteArray'])
    imageProcessed              = pyqtSignal(['QPixmap'])

    telemetryReceived          = pyqtSignal(['QString'])
    telemetryProcessed         = pyqtSignal([list])
    missionTimeUpdated         = pyqtSignal(['QString'])
    ########################################################

    
    def __init__(self):
        super(SerialProcess,self).__init__()
        self.serial = MySerial()
        self.device = self.serial.openSerialPort()
        self.watchdog=   QTimer(self)
        self.frameTimer = QTimer(self)


        self.setWatchdogInterval(15)
       
        self.frameTimer.setInterval(1000.0)
        self.frameTimer.setTimerType(Qt.PreciseTimer)
        self.frameTimer.timeout.connect(self.readFrames)
        self.frameTimer.start()

        self.imageReceived.connect(self.receiveImage)
        self.telemetryReceived.connect(self.receiveTelemetry)
        time.sleep(0.1)
        self.connectDevice()

    def deviceAvailable(self):
        return self.device is not None


    def watchdogInterval(self):
        return self.watchdog.interval()

    
    def writeData(self,data):
        bytes = 0
        
        bytes = self.device.write(data)

        if(bytes>0):
            self.tx.emit()
        
        self.device.flush()
        #print(self.device)
        self.device.waitForBytesWritten()
        return bytes
    ## Slots
    ########################################################
    
    @pyqtSlot()
    def startReading(self):
        LOG("Serial Manager Initialized",self.isLogLocal)



        #self.watchdog.moveToThread(QThread.currentThread())
        #self.frameTimer.moveToThread(QThread.currentThread())
    
    @pyqtSlot()
    def connectDevice(self):
        #self.disconnectDevice()

        self.setDevice(self.serial.openSerialPort())

        if(self.deviceAvailable()):
            mode = QIODevice.ReadWrite
            if(self.device.open(mode)):
                self.device.readyRead.connect(self.onDataReceived)
                LOG("Device Opened Successfully",self.isLogLocal)

            else:
                LOG("Failed to Open Device",self.isLogLocal)
                self.disconnectDevice()

            self.connectedChanged.emit()
            self.deviceChanged.emit()
            
    @pyqtSlot()
    def toggleConnection(self):
        pass

    @pyqtSlot()
    def disconnectDevice(self):
        if(self.deviceAvailable()):
            try: self.device.disconnect()
            except: pass
            self.device = None
            self.receivedBytes = 0
            self.dataBuffer = QByteArray()
            
            self.connectedChanged.emit()
            self.deviceChanged.emit()

    @pyqtSlot(int,bool)
    def setWriteEnabled(self,enabled):
        pass

    @pyqtSlot(int,int)
    def setMaxBufferSize(self,maxBufferSize):
        pass

    @pyqtSlot(int,int)
    def setWatchdogInterval(self,interval=15):
        self.watchdog.setInterval(interval)
        self.watchdog.setTimerType(Qt.PreciseTimer)
        self.watchdogIntervalChanged.emit()
    
    @pyqtSlot('QByteArray')
    def receiveImage(self,imageString):
        s = imageString
      #  sList = s.split(',')
      #  while("" in sList):
      #      sList.remove("")
       # x,y,color = sList[0:3]
        #sList = sList[3:]

       # print(x,y)
        #print(len(sList))

        #iList = list(map(int,sList))
        #shape = tuple(map(int,[x,y]))

        #MatImg = np.reshape(iList,shape)
        #im = Image.frombytes('RGB',shape,bytes(s[7:],"utf-8"),'raw')
        im = Image.open(io.BytesIO(s))
        #im = Image.fromarray(np.uint8(MatImg))
        im.show()
        im.save("image" + str(self.numImagesReceived)+".png")
        MatImg = np.array(im)
        qimage = QImage(MatImg, MatImg.shape[1], MatImg.shape[0], QImage.Format_RGB888)

        pixm = QPixmap.fromImage(qimage)
        #pixm.setDevicePixelRatio(.1)
        self.imageProcessed.emit(pixm)

    @pyqtSlot('QString')
    def receiveTelemetry(self,telemetryString):
        s = str(telemetryString)
        sList = s.split(',')
        while("" in sList):
            sList.remove("")
        tel = list(map(float,sList))
        #LOG(tel,self.isLogLocal)
        self.telemetryProcessed.emit(tel)


        mt = "T+ " + str(tel[0])
        self.missionTimeUpdated.emit(mt)

    @pyqtSlot()
    def readFrames(self):

        LOG("ReadFrames",self.isLogLocal)



        #
        startNormal = "UAH Charger RocketWorks"
        endNormal = "UAH Charger RocketWorks End"

        startImg = bytes("Image",'utf-8')
        endImg = bytes("Image End",'utf-8')
        
       
        strBuff = str(self.dataBuffer)
        #strBuff = ",,ImageII*\x00\x08\x00\x00\x00\n\x00\x00\x01\x04\x00\x01\x00\x00\x00-\x03\x00\x00\x01\x01\x04\x00\x01\x00\x00\x00b\x02\x00\x00\x02\x01\x03\x00\x03\x00\x00\x00\x86\x00\x00\x00\x03\x01\x03\x00\x01\x00\x00\x00\x01\x00\x00\x00\x06\x01\x03\x00\x01\x00\x00\x00\x02\x00\x00\x00\x11\x01\x04\x00\x01\x00\x00\x00\x8c\x00\x00\x00\x15\x01\x03\x00\x01\x00\x00\x00\x03\x00\x00\x00\x16\x01\x04\x00\x01\x00\x00\x00b\x02\x00\x00\x17\x01\x04\x00\x01\x00\x00\x00\xae\xb3\x16\x00\x1c\x01\x03\x00\x01\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x08\x00\x08\x00\x08\x00L\x87\xd5L\x87\xd5L\x87\xd5N\x89\xd7L\x8a\xd7L\x8a\xd7K\x89\xd6J\x88\xd5K\x89\xd6L\x8a\xd7K\x89\xd6J\x88\xd5L\x87\xd5M\x88\xd6N\x89\xd7N\x89\xd7M\x88\xd6N\x89\xd7N\x89\xd7N\x89\xd7N\x89\xd7O\x8a\xd8O\x8a\xd8N\x89\xd7N\x89\xd7N\x89\xd7N\x89\xd7O\x8a\xd8O\x8a\xd8N\x89\xd7N\x89\xd7P\O\x8a\xd8O\x8a\xd8N\x89\xd7N\x89\xd7P\x8b\xd9N\x87\xd6N\x87\xd6O\x88\xd7P\x89\xd8P\x89\xd8O\x88\xd7N\x89\xd7M\x88\xd6N\x89\xd7O\x8a\9P\x8b\xd9O\x8a\xd8N\x89\xd5P\x8b\xd7Qxd8O\x8a\xd8N\x89\xd7N\x89\xd7P\x8b\xd9P\x8b\xd9O\x8a\xd8N\x89\xd5P\x8b\xd7Q\x8c\xd8Q\x8c\xd8Q\x8a\xd7P\x89\xd6P\x89\xd6Q\x8a\xd7O\x2176,4.876302361532091,-4.48723729265888\xd5P\x89\xd6Q\x8a\xd7Q\x8a,UAH Charger RocketWorks,22.0,5,0,5.354944287012176,4.876302361532091,-4.487237292658957,0.032817569596944287012176,4.876302361532091,-4.487278448,0.0065896653801972415,0.03238312012398757,UAH Charger RocketWorks End,UAH Charger RocketWorks,22.5,5,0,5.354944287012176,4.876,0,5.354944287012176,4.876302361532091302361532091,-4.487237292658957,0.03281756959678448,0.0065896653801972415,0.03238312012398757,UAH Charger RocketWorks End,UAH Charges,23.5,5,0,5.354944287012176,4.8763023r RocketWorks,23.0,5,0,5.354944287012176,4.876302361532091,-4.487237292658957,0.03281756959678448,0.0065896653801972415,0.0323831201cketWorks,24.0,5,0,5.3549442870'      2398757,UAH Charger RocketWorks End,UAH Charger RocketWorks,23.5,5,0,5.354944287012176,4.876302361532091,-4.487237292658957,0.03281756959678448,0.0065896653801972415,0.03238312012398757,UAH Charger RocketWorks End,UAH Charger RocketWorks,24.0,5,0,5.3549442870"
        #strBuff = strBuff[3:-1]
        while((startNormal in strBuff) and (endNormal in strBuff)):
           #self.feedWatchdog()
            #print("ORIGINAL BUFFER: " + strBuff + "\n")
            buffer = strBuff
            sIndex = buffer.index(startNormal)

            buffer = buffer[sIndex+len(startNormal)+1:]
            LOG("Buffer1: " + buffer,self.isLogLocal)
            if(not endNormal in buffer):
                break

            fIndex = buffer.index(endNormal)
            buffer = buffer[0:fIndex-1]
            LOG("Buffer2: " + buffer,self.isLogLocal)
            if(buffer != ""):
                LOG("SIndex:  " + str(sIndex))
                self.telemetryReceived.emit(buffer)
                self.dataBuffer = self.dataBuffer[0:sIndex-2]+self.dataBuffer[fIndex+len(endNormal)+sIndex+len(startNormal):]
                
                strBuff = str(self.dataBuffer)
                LOG(self.dataBuffer,self.isLogLocal)
                print("")
                #print(str(self.dataBuffer[0:sIndex-2]+self.dataBuffer[fIndex+len(endNormal)+sIndex+len(startNormal):]))
                print(sIndex)
                print(fIndex)
                print("")
            #strBuff = str(self.dataBuffer)
        while((self.dataBuffer.contains(startImg)) and (self.dataBuffer.contains(endImg))):
            self.feedWatchdog()
            buffer = self.dataBuffer
            sIndex = self.dataBuffer.indexOf(startImg)

            buffer = buffer[sIndex+len(startImg):]
           # LOG("Buffer1: " + str(buffer) + "\n",self.isLogLocal)
            if(not endImg in buffer):
                break

            fIndex = buffer.indexOf(endImg)
            buffer = buffer[0:fIndex]
            #LOG("Buffer2: " + str(buffer) + "\n",self.isLogLocal)
            if(buffer != ""):
                self.imageReceived.emit(buffer)
                self.dataBuffer = self.dataBuffer[0:sIndex-1]+self.dataBuffer[fIndex+len(endImg)+sIndex+len(startImg)+1:]
                #LOG(self.dataBuffer,self.isLogLocal)





    @pyqtSlot()
    def feedWatchdog(self):
        self.watchdog.stop()
        self.watchdog.start()
        #print("Timer Thread: " + str(int(QThread.currentThreadId())))

    @pyqtSlot()
    def onDataReceived(self):

        #if(self.device == None):
         #   self.disconnectDevice()
        
        data = self.device.readAll()
        byt = len(data)

        self.feedWatchdog()
        
        s = data
    

        self.dataBuffer.append(s)

        self.receivedBytes += byt

        
        self.receivedBytesChanged.emit()
        self.dataReceived.emit(data)
        self.rx.emit()
       # print("Received Data:   " + str(s))
        #print("")
       # print("")


    @pyqtSlot()
    def clearTempBuffer(self):
        self.dataBuffer = QByteArray()

    @pyqtSlot()
    def onWatchdogTriggered(self):
        self.clearTempBuffer()

    @pyqtSlot(int,'QIODevice')
    def setDevice(self,device):
        self.disconnectDevice()
        self.device = device
        self.deviceChanged.emit()
    
    
class SerialManager(QObject):
    runSerialProcess = pyqtSignal()
    telemetryProcessed = pyqtSignal([list])
    missionTimeUpdated = pyqtSignal(['QString'])
    imageProcessed = pyqtSignal(['QPixmap'])

    def __init__(self,port):
        super(SerialManager,self).__init__()
        self.serP = SerialProcess()
         


        self.thread = QThread()
        self.serP.moveToThread(self.thread)
        

        self.serP.serial.port = port
        self.serP.telemetryProcessed.connect(self.receiveTelemetryFromSerial,type=Qt.DirectConnection)
        self.serP.imageProcessed.connect(self.receiveImageFromSerial,type=Qt.DirectConnection)
        self.serP.missionTimeUpdated.connect(self.passOnMissionTime,type=Qt.DirectConnection)
        self.runSerialProcess.connect(self.serP.startReading,type=Qt.DirectConnection)
        
        self.thread.start()
        self.runSerialProcess.emit()
       


    @pyqtSlot('QPixmap')
    def receiveImageFromSerial(self,image):
        self.image = image
        self.imageProcessed.emit(self.image)


    @pyqtSlot(list)
    def receiveTelemetryFromSerial(self,telList):
        self.telList = telList
        self.telemetryProcessed.emit(telList)


    @pyqtSlot('QString')
    def passOnMissionTime(self,time):
        self.missionTimeUpdated.emit(time)
