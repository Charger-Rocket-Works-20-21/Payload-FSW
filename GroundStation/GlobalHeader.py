import sys
import io
import time
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtSerialPort import *
from PyQt5 import uic
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

import serial.tools.list_ports
import serial
from serial import *
from PIL import Image
import numpy as np


isLogGlobal = True
isSimulateSerial = False

def LOG(message, isLogEnabled=True):
    if(isLogEnabled):
        print(message)
        
