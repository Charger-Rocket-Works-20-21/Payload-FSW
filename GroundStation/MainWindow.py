import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess

isLogGlobal = False

def LOG(message, isLogEnabled=True):
    if(isLogEnabled):
        print(message)
        


class window(QWidget):
    isLogLocal = isLogGlobal;

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
      LOG("Creating Serial", self.isLogLocal)
      self.ser = SerialProcess()
      self.ser.start_process()



      
def main():
   app = QApplication(sys.argv)
   ex = window()
   ex.show()
   sys.exit(app.exec_())
if __name__ == '__main__':
   main()
