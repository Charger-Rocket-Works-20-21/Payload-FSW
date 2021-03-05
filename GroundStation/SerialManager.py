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
    isLogLocal = isLogGlobal

    dataBuffer = QByteArray()
    device = None
    serial = None
    watchdog = QTimer()
    receivedBytes = 0
    numImagesReceived = 0
    frameTimer = QTimer()
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
        LOG("Serial Manager Initialized",self.isLogLocal)
        self.setWatchdogInterval(15)
        self.serial = MySerial()
        self.device = self.serial.openSerialPort()

        self.frameTimer.setInterval(100.0)
        self.frameTimer.setTimerType(Qt.PreciseTimer)
        self.frameTimer.timeout.connect(self.readFrames)
        self.frameTimer.start()

        self.imageReceived.connect(self.receiveImage)
        self.telemetryReceived.connect(self.receiveTelemetry)


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
        #LOG("ReadFrames",self.isLogLocal)
        startNormal = "UAH Charger RocketWorks"
        endNormal = "UAH Charger RocketWorks End"

        startImg = bytes("Image",'utf-8')
        endImg = bytes("Image End",'utf-8')

        strBuff = str(self.dataBuffer)
        while((startNormal in strBuff) and (endNormal in strBuff)):
            buffer = strBuff
            sIndex = strBuff.index(startNormal)

            buffer = buffer[sIndex+len(startNormal)+1:]
            LOG("Buffer1: " + buffer,self.isLogLocal)
            if(not endNormal in buffer):
                break

            fIndex = buffer.index(endNormal)
            buffer = buffer[0:fIndex-1]
            LOG("Buffer2: " + buffer,self.isLogLocal)
            if(buffer != ""):
                self.telemetryReceived.emit(buffer)
                self.dataBuffer = self.dataBuffer[0:sIndex-1]+self.dataBuffer[fIndex+len(endNormal)+sIndex+len(startNormal)+1:]
                strBuff = str(self.dataBuffer)
                LOG(self.dataBuffer,self.isLogLocal)
                print("")


        while((self.dataBuffer.contains(startImg)) and (self.dataBuffer.contains(endImg))):
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
                #(self.dataBuffer,self.isLogLocal)

        


    @pyqtSlot()
    def feedWatchdog(self):
        self.watchdog.stop()
        self.watchdog.start()

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
        print("Received Data:   " + str(s))
        print("")
        print("")

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
    
    
