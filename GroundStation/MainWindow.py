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

##      self.ser2 = SerialProcess()
##      self.ser2.serial.port = "COM2"
##
##      self.ser2.connectDevice()
##
##
##      self.ser2.serial.serialPort.writeData(bytes('abc','utf-8'))
##      self.ser2.serial.serialPort.flush()
##
#      ser2 = SerialProcess()
#      ser2.serial.port = "COM2"
#
#      ser2.connectDevice()
#
 #     ser2.serial.serialPort.write(bytes('1','utf-8'))
  #    ser2.serial.serialPort.flush()
   #   #ser2.connectDevice()
   #   ser2.serial.serialPort.write(bytes('2','utf-8'))
   #   ser2.serial.serialPort.flush()
    #  #ser2.connectDevice()
    #  ser2.serial.serialPort.write(bytes('3','utf-8'))
    #  ser2.serial.serialPort.flush()

      if(isSimulateSerial):
          self.p = QProcess()
          
          self.p.start("C:\\Users\\quaz9\\AppData\\Local\\Programs\\Python\\Python39\\python",["SimulateSerial.py"])
          self.p.waitForFinished(-1)  





      
def main():
   app = QApplication(sys.argv)
   ex = window()
   ex.show()
   sys.exit(app.exec_())
   
if __name__ == '__main__':
   main()
