from GlobalHeader import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess






class window(QWidget):
    isLogLocal = isLogGlobal

    # Serial Manager
    ser = None

    # CSV Manger
    csvFile = None
    
    
    def __init__(self, parent = None):
      super(window, self).__init__(parent)
      self.setWindowTitle("UAH Ground Station")

            

      self.csvFile = CSVFileManager()
      
      self.ser = SerialProcess()
      self.ser.serial.port = "COM1"
      self.ser.connectDevice()


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
