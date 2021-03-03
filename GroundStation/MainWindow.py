from GlobalHeader import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess






class window(QWidget):
    isLogLocal = isLogGlobal

    # Serial Manager
    ser = None

    # CSV Manger
    csvFile = None
    
    def windowTitleChanged(self,e):
        
        super(window,self).keyPressEvent(e)
    
    def __init__(self, parent = None):
      super(window, self).__init__(parent)
      self.setWindowTitle("UAH Ground Station")

      

      self.csvFile = CSVFileManager()
      
      self.ser = SerialProcess()
      self.ser.serial.port = "COM1"
      self.ser.connectDevice()

      self.ser2 = SerialProcess()
      self.ser2.serial.port = "COM2"

      self.ser2.connectDevice()


      self.ser.serial.serialPort.writeData(bytes('abc','utf-8'))



      
def main():
   app = QApplication(sys.argv)
   ex = window()
   ex.show()
   sys.exit(app.exec_())
   
if __name__ == '__main__':
   main()
