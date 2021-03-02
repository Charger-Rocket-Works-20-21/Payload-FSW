"""CSV Module: Handles all CSV operations, from opening to saving.
@author: Nathan Ulmer
created: 3/1/2021
"""

# Use Python's built-in CSV module
import csv

#Import PyQt Stuff so we can attach connections
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *



class CSVFileManager(QObject):

    filePath = "default.csv"
    
    def __init__(self,name="default.csv"):
        self.filePath = name
        

    @pyqtSlot(list)    
    def writeLine(self,lineList):
        if (type(lineList) == type("")):
            print(str(lineList))
            return
        with open(self.filePath,'a',newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(lineList)


    
if __name__ == "__main__":
    test = CSVFileManager()

    testList = [12,13,14,15,16]
    testList2 = ["A",'b',"C",'d']

    test.writeLine(testList)
    test.writeLine(testList2)
