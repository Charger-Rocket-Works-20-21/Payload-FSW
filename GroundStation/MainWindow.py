from GlobalHeader import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess,SerialManager






class window(QMainWindow):
    isLogLocal = isLogGlobal

    # Serial Manager
    ser = None

    # CSV Manger
    csvFile = None
    
    
    def __init__(self, parent = None):
      super(window, self).__init__(parent)
      uic.loadUi('main.ui',self)
      self.setWindowTitle("UAH Ground Station")

      

      self.csvFile = CSVFileManager()
      
      
      self.ser = SerialManager("COM1")
      
     

      self.ser.telemetryProcessed.connect(self.csvFile.writeLine)

      self.ser.missionTimeUpdated.connect(self.missionTime.setText)
      self.ser.imageProcessed.connect(self.image.setPixmap)

      print("Main Thread: " + str(int(QThread.currentThreadId())))

      if(isSimulateSerial):
          self.p = QProcess()
          self.p.start("C:\\Users\\quaz9\\AppData\\Local\\Programs\\Python\\Python39\\python",["SimulateSerial.py"])
          #self.p.waitForFinished(-1)  





      
def main():
   app = QApplication(sys.argv)


   ex = window()
   ex.show()
   sys.exit(app.exec_())
   
if __name__ == '__main__':
   main()
