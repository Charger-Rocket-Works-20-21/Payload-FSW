from GlobalHeader import *


portlist = [comport.portName() for comport in QSerialPortInfo().availablePorts()]

for port in portlist: print(port)



class MySerial(QObject):
    isLogLocal = False
    baudRate = 0
    settings = QSettings()
    parity = 0
    dataBits = 0
    stopBits = 0
    flowControl = 0
    portList = []
    baudRateList = []
    serialPort = QSerialPort()
    port = "COM2"
    
    def __init__(self):
        super(MySerial,self).__init__()
        LOG("Serial Device Initialized")


    def openSerialPort(self):
        try: self.serialPort.close()
        except: pass

        self.serialPort = QSerialPort(self.port)
        self.serialPort.setBaudRate(self.baudRate)
        time.sleep(0.001)
        return self.serialPort

    @pyqtSlot('QString')
    def setPort(self,name):
        
        self.port = name
        self.serialPort.setPortName(name)
        

    @pyqtSlot('QString')
    def setBaud(self,baud):
        self.baudRate = int(baud)
        a = self.serialPort.setBaudRate(self.baudRate)
        LOG("Work? " +str(a))
        time.sleep(0.001)




class SerialProcess(QObject):
    ## Instance Vars
    isLogLocal = False

    dataBuffer = QByteArray()
    device = None
    serial = MySerial()
    watchdog = None
    receivedBytes = 0
    numImagesReceived = 0
    frameTimer = None
    lockWhile = False
    lockRead = False

    droppedPackets = 0
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
    packetDropped              = pyqtSignal([int])
    imageFailed                = pyqtSignal()
    comListUpdated             = pyqtSignal(['QStringList'])
    ########################################################

    
    def __init__(self):
        super(SerialProcess,self).__init__()
        self.serial = MySerial()
        self.watchdog=   QTimer(self)
        self.frameTimer = QTimer(self)


        self.setWatchdogInterval(5)
       
        self.frameTimer.setInterval(200.0)
        self.frameTimer.setTimerType(Qt.PreciseTimer)
        self.frameTimer.timeout.connect(self.readFrames)
        self.frameTimer.start()

        self.imageReceived.connect(self.receiveImage)
        self.telemetryReceived.connect(self.receiveTelemetry)


        portlist = [comport.portName() for comport in QSerialPortInfo().availablePorts()]

        self.comListUpdated.emit(portlist)


    def deviceAvailable(self):
        return self.device is not None


    def watchdogInterval(self):
        return self.watchdog.interval()

    
    @pyqtSlot('QString')
    def writeData(self,data):
        if(self.device):
            byt = 0
       
            byt = self.device.write(data)

            if(byt>0):
                self.tx.emit()
            
            self.device.flush()
            
        
            self.device.waitForBytesWritten()
            print(str(data))
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
                LOG(self.device.baudRate())
                self.disconnectDevice()

            self.connectedChanged.emit()
            self.deviceChanged.emit()
            
    @pyqtSlot()
    def toggleConnection(self):
        if(self.deviceAvailable()):
            self.disconnectDevice()
        else:
            self.connectDevice()

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
        #try:
            s = imageString
        
            im = Image.open(io.BytesIO(s))
            #im = Image.fromarray(np.uint8(MatImg))
            #im.show()
            im.save("image" + str(self.numImagesReceived)+".png")
            MatImg = np.array(im)
            qimage = QImage()



            dtype = 'linear'
            format = 'fullframe'
            fov = 120
            pfov = 120
            #impath = "image" + str(self.numImagesReceived)+".png"
            #obj = Defisheye(impath, dtype=dtype, format=format, fov=fov, pfov=pfov)
            #obj.convert(impath)

            qimage.load("image" + str(self.numImagesReceived)+".png",format='PNG')
            #qimage.save("QIMAGE.png")
            pixm = QPixmap.fromImage(qimage)



            self.imageProcessed.emit(pixm)
            self.numImagesReceived = self.numImagesReceived +1

            

            if(self.numImagesReceived ==3):
                
                images = []
                im0 = cv2.imread("image0.png")
                images.append(im0)
                im1 = cv2.imread("image1.png")
                images.append(im1)
                im2 = cv2.imread("image2.png")
                images.append(im2)

                stitcher = cv2.createStitch() if imutils.is_cv3() else cv2.Stitcher_create()
                (status,stitched) = stitcher.stitch(images)

                if status == 0:
                    cv2.imwrite("panorama.png",stitched)

                    
                else:
                    LOG("Panorama processing failed, defaulting to failsafe method")
                    LOG(status)

                    im0 = Image.open("image0.png")
                    im1 = Image.open("image1.png")
                    im2 = im
                    
                    im0_size = im0.size
                    im1_size = im1.size
                    im2_size = im2.size

                    pan_w = im0_size[0] + im1_size[0] + im2_size[0]
                    pan_h = max(im0_size[1], im1_size[1], im2_size[1])
                    panorama = Image.new('RGB',(pan_w,pan_h),(250,250,250))
                    panorama.paste(im0,(0,0))
                    panorama.paste(im1,(im0_size[0],0))
                    panorama.paste(im2,(im0_size[0]+im1_size[0],0))
                    panorama.save("panorama.png")
                    """


                """

                qimage2 = QImage()
                qimage2.load("panorama.png",format='PNG')
            
                pixm2 = QPixmap.fromImage(qimage2)
                self.imageProcessed.emit(pixm2)


       #except:
        #    LOG("IMAGE RECEIVE FAILED, UNABLE TO PROCESS",True)
            
        #    self.imageFailed.emit()

    @pyqtSlot('QString')
    def receiveTelemetry(self,telemetryString):

        try:
            s = str(telemetryString)
            sList = s.split(',')
            while("" in sList):
                sList.remove("")

        
            tel = list(map(float,sList))
            #LOG(tel,self.isLogLocal)
            self.telemetryProcessed.emit(tel)


            mt = "T+ " + str(tel[1])
            self.missionTimeUpdated.emit(mt)
        except:
            LOG("PACKET DROPPED, UNABLE TO PROCESS",True)
            self.droppedPackets = self.droppedPackets+1
            self.packetDropped.emit(self.droppedPackets)

    @pyqtSlot()
    def readFrames(self):
        self.portlist = [comport.portName() for comport in QSerialPortInfo().availablePorts()]

        self.comListUpdated.emit(self.portlist)
        #LOG("ReadFrames",self.isLogLocal)
        #LOG(str(self.dataBuffer),self.isLogLocal)
        

        #
        startNormal = "UAH Charger RocketWorks"
        endNormal = "UAH Charger RocketWorks End"

        startImg = bytes("Image",'utf-8')
        endImg = bytes("Image End",'utf-8')
        #LOG(self.dataBuffer.contains(endImg))
        #self.dataBuffer = self.dataBuffer
        while((self.dataBuffer.contains(startImg)) and (self.dataBuffer.contains(endImg))):
            
            #self.feedWatchdog()
            buffer = self.dataBuffer
            sIndex = self.dataBuffer.indexOf(startImg)

            buffer = buffer[sIndex+len(startImg):]
            LOG("Buffer1: " + str(buffer) + "\n",self.isLogLocal)
            if(not endImg in buffer):
                break

            fIndex = buffer.indexOf(endImg)
            buffer = buffer[0:fIndex]
            LOG("Buffer2: " + str(buffer) + "\n",self.isLogLocal)
            if(buffer != ""):
                self.isLogLocal = True
                temp = QByteArray(buffer)
                time.sleep(0.1)
                self.imageReceived.emit(temp)
                self.dataBuffer = self.dataBuffer[0:sIndex-1]+self.dataBuffer[fIndex+len(endImg)+sIndex+len(startImg)+1:]
                LOG(self.dataBuffer,self.isLogLocal)
                self.isLogLocal = False

       
        strBuff = str(self.dataBuffer)

        while((startNormal in strBuff) and (endNormal in strBuff)):
           
            #self.feedWatchdog()
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
                LOG("SIndex:  " + str(sIndex),self.isLogLocal)
                self.telemetryReceived.emit(buffer)
                self.dataBuffer = self.dataBuffer[0:sIndex-2]+self.dataBuffer[fIndex+len(endNormal)+sIndex+len(startNormal):]
                
                strBuff = str(self.dataBuffer)
                LOG("STR BUFF:   " + strBuff,self.isLogLocal)

            #strBuff = str(self.dataBuffer)





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
        #print(str(s))
        self.receivedBytesChanged.emit()
        self.dataReceived.emit(data)
        self.rx.emit()



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
    connectionToggled = pyqtSignal()

    comListUpdated = pyqtSignal(['QStringList'])
    comPortSelected = pyqtSignal(['QString'])
    baudRateSelected = pyqtSignal(['QString'])
    def __init__(self):
        super(SerialManager,self).__init__()
        self.serP = SerialProcess()
         


        self.thread = QThread()
        self.serP.moveToThread(self.thread)
        
        self.serP.telemetryProcessed.connect(self.receiveTelemetryFromSerial,type=Qt.DirectConnection)
        self.serP.imageProcessed.connect(self.receiveImageFromSerial,type=Qt.DirectConnection)
        self.serP.missionTimeUpdated.connect(self.passOnMissionTime,type=Qt.DirectConnection)
        self.runSerialProcess.connect(self.serP.startReading,type=Qt.DirectConnection)
        self.connectionToggled.connect(self.serP.toggleConnection,type = Qt.DirectConnection)
        self.serP.comListUpdated.connect(self.comListUpdatedConnect,type=Qt.DirectConnection)
        self.comPortSelected.connect(self.serP.serial.setPort,type=Qt.DirectConnection)
        self.baudRateSelected.connect(self.serP.serial.setBaud,type=Qt.DirectConnection)




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

    @pyqtSlot()
    def toggleConnection(self):
        self.connectionToggled.emit()


    @pyqtSlot('QStringList')
    def comListUpdatedConnect(self,comlist):
        self.comListUpdated.emit(comlist)