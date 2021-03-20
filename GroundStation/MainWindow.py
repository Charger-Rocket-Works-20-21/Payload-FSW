from GlobalHeader import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess,SerialManager






class window(QMainWindow):
   isLogLocal = isLogGlobal

    # Serial Manager
   ser = None

    # CSV Manger
   csvFile = None
   
   times = []
   alt = []
   ax = []
   ay = []
   az = []

   imgCounter = 0
   portlist = []
   def __init__(self, parent = None):
      super(window, self).__init__(parent)
      uic.loadUi('main.ui',self)

      self.csvFile = CSVFileManager()
      
      
      self.ser = SerialManager()
      
     

      self.ser.telemetryProcessed.connect(self.csvFile.writeLine)
      self.ser.telemetryProcessed.connect(self.handlePlot)
      self.ser.missionTimeUpdated.connect(self.missionTime.setText)
      self.ser.imageProcessed.connect(self.displayImage)
    
      self.ser.comListUpdated.connect(self.updateComList, type=Qt.DirectConnection)
      
      self.actionToggle_Serial_Settings_Dock.triggered.connect(self.toggleSerialSettingsVisible)
      self.connectButton.released.connect(self.ser.toggleConnection)

      self.setWindowTitle("UAH Ground Station")
      self.comboBox_portList.textActivated.connect(self.ser.serP.serial.setPort)

      self.comboBox_baud.addItems(["115200","9600"])
      self.comboBox_baud.textActivated.connect(self.ser.serP.serial.setBaud)
      ##
      self.pen = pg.mkPen(color=(255, 0, 0), width=4)
      self.graphWidget = pg.PlotWidget()
      #self.setCentralWidget(self.graphWidget)

      hour = [1,2,3,4,5,6,7,8,9,10]
      temperature = [30,32,34,32,33,31,29,32,35,45]

      self.graphWidget.setBackground('w')
      self.graphWidget.plot(hour, temperature)
      ##
      

      ##

      self.graphWidget2 = pg.PlotWidget()
      self.graphWidget2.setBackground('w')
      self.graphWidget2.setTitle("Altitude",size = '20pt')
      styles = {'color':'k', 'font-size':'14px'}
      self.graphWidget2.setLabel('left',"Altitude (m)", **styles)
      self.graphWidget2.setLabel('bottom',"Time (s)", **styles)
      self.graphWidget2.addLegend()
      self.graphWidget2.showGrid(x=True,y=True)
      self.alt_data = self.graphWidget2.plot([0],[0],name="Alt",pen=self.pen)
      ##

      ##
      self.pen = pg.mkPen(color=(255, 0, 0), width=4)
      self.pen2 = pg.mkPen(color=(0, 255, 0), width=4)
      self.pen3 = pg.mkPen(color=(0, 0, 255), width=4)

      self.graphWidget3 = pg.PlotWidget()
      self.graphWidget3.setBackground('w')
      self.graphWidget3.setTitle("Acceleration",size = '20pt')
      styles = {'color':'k', 'font-size':'14px'}
      self.graphWidget3.setLabel('left',"Acceleration (m/s^2)", **styles)
      self.graphWidget3.setLabel('bottom',"Time (s)", **styles)
      self.graphWidget3.addLegend()
      self.graphWidget3.showGrid(x=True,y=True)
      self.ax_data = self.graphWidget3.plot([0],[0],name="X Acceleration",pen=self.pen)
      self.ay_data = self.graphWidget3.plot([0],[0],name="Y Acceleration",pen=self.pen2)
      self.az_data = self.graphWidget3.plot([0],[0],name="Z Acceleration",pen=self.pen3)
      ##


      ##
      self.graphWidget4 = pg.PlotWidget()
      self.graphWidget4.setBackground('w')
      ##
      self.tab_3.layout().replaceWidget(self.widget,  self.graphWidget)
      self.tab_3.layout().replaceWidget(self.widget_2,  self.graphWidget2)
      self.tab_3.layout().replaceWidget(self.widget_3,  self.graphWidget3)
      self.tab_3.layout().replaceWidget(self.widget_4,  self.graphWidget4)

 

      print("Main Thread: " + str(int(QThread.currentThreadId())))

      if(isSimulateSerial):
          self.p = QProcess()
          self.p.start("C:\\Users\\quaz9\\AppData\\Local\\Programs\\Python\\Python39\\python",["SimulateSerial.py"])
          #self.p.waitForFinished(-1)  

   @pyqtSlot(bool)
   def toggleSerialSettingsVisible(self,stuff):
      self.dockSerialSettings.setVisible(not self.dockSerialSettings.isVisible())


   @pyqtSlot(list)
   def handlePlot(self,telList):
      
      self.times.append(telList[0])
      self.alt.append(telList[2])
      
      self.ax.append(telList[3])
      self.ay.append(telList[4])
      self.az.append(telList[5])

      self.alt_data.setData(self.times,self.alt)

      self.ax_data.setData(self.times,self.ax)
      self.ay_data.setData(self.times,self.ay)
      self.az_data.setData(self.times,self.az)


   @pyqtSlot('QPixmap')
   def displayImage(self,image):
      if(self.imgCounter == 0):
         w = self.image0.width()
         h = self.image0.height()
         self.image0.setPixmap(image.scaled(w,h,Qt.KeepAspectRatio))
      if(self.imgCounter == 1):
         w = self.image1.width()
         h = self.image1.height()
         self.image1.setPixmap(image.scaled(w,h,Qt.KeepAspectRatio))    
      if(self.imgCounter == 2):
         w = self.image2.width()
         h = self.image2.height()
         self.image2.setPixmap(image.scaled(w,h,Qt.KeepAspectRatio))
      if(self.imgCounter == 3):
         w = self.image3.width()
         h = self.image3.height()
         self.image3.setPixmap(image.scaled(w,h,Qt.KeepAspectRatio))
      self.imgCounter = self.imgCounter +1


   @pyqtSlot('QStringList')
   def updateComList(self,comlist):
      if(comlist != self.portlist):
         self.comboBox_portList.clear()
         self.comboBox_portList.addItems(comlist)
         self.portlist = comlist
         LOG("A")


def main():
   app = QApplication(sys.argv)


   ex = window()
   ex.show()
   sys.exit(app.exec_())




   
if __name__ == '__main__':
   main()
