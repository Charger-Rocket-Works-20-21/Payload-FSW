
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import serial.tools.list_ports



portlist = [comport.name + ' - ' + comport.description for comport in serial.tools.list_ports.comports()]

#print('Currently Open Ports:')
for port in portlist: print(port)



class SerialProcess(QObject):

    def __init__(self):
        super(SerialProcess,self).__init__()



    def message(self,s):
        print(s)

    def start_process(self):
        self.message("Starting Serial Listener Process.")
        self.p = QProcess()
        self.p.readyReadStandardOutput.connect(self.handle_stdout)
        self.p.readyReadStandardError.connect(self.handle_stderr)
        self.p.stateChanged.connect(self.handle_state)
        self.p.finished.connect(self.process_finished)
        self.p.start("C:\\Users\\quaz9\\AppData\\Local\\Programs\\Python\\Python39\\python",['C:\\Users\\quaz9\\Documents\\MAE490\\Payload-FSW\\GroundStation\\main.py'])
        print("NOLO")
        


    def handle_stdout(self):
        #print("AAAAAA")
        data = self.p.readAllStandardOutput()
        stdout = bytes(data).decode("utf8")
        self.message(stdout)
        
    def handle_stderr(self):
        data = self.p.readAllStandardError()
        stderr = bytes(data).decode("utf8")
        self.message(stderr)

    def handle_state(self, state):
        states = {   
            QProcess.NotRunning: 'Not running',
            QProcess.Starting: 'Starting',
            QProcess.Running: 'Running',
        }
        print(str(self.p.error()))
        state_name = states[state]
        self.message(f"State changed: {state_name}")
        

    def process_finished(self):
            self.message("Process finished.")
            self.p = None        



#print("Hello")
