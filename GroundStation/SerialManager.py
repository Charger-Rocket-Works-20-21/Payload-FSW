from GlobalHeader import *

import serial.tools.list_ports
import serial
from serial import *


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

    dataBuffer = []
    device = None
    serial = None
    watchdog = QTimer()
    receivedBytes = 0
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
    frameReceived               = pyqtSignal(['QByteArray'])
    ########################################################

    
    def __init__(self):
        super(SerialProcess,self).__init__()
        LOG("Serial Manager Initialized",self.isLogLocal)
        self.setWatchdogInterval(15)
        self.serial = MySerial()
        self.device = self.serial.openSerialPort()
        


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
            self.dataBuffer = []
            
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
    

    @pyqtSlot()
    def readFrames(self):
        pass

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

        self.dataBuffer.append(data)

        self.receivedBytes += byt

        self.receivedBytesChanged.emit()
        self.dataReceived.emit(data)
        self.rx.emit()
        print("Received Data:   " + str(data))

    @pyqtSlot()
    def clearTempBuffer(self):
        self.dataBuffer.clear()

    @pyqtSlot()
    def onWatchdogTriggered(self):
        self.clearTempBuffer()

    @pyqtSlot(int,'QIODevice')
    def setDevice(self,device):
        self.disconnectDevice()
        self.device = device
        self.deviceChanged.emit()
    
    
