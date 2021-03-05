import sys
import time
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtSerialPort import *



isLogGlobal = True
isSimulateSerial = True

def LOG(message, isLogEnabled=True):
    if(isLogEnabled):
        print(message)
        
