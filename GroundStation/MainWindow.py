from GlobalHeader import *


from CSVManager import CSVFileManager
from SerialManager import SerialProcess,SerialManager

from OpenGL.GL import *

from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL import GLUT
import math




class window(QMainWindow):
   isLogLocal = isLogGlobal

    # Serial Manager
   ser = None

    # CSV Manger
   csvFile = None
   
   times = []
   alt = []
   fs = []
   ax = []
   ay = []
   az = []
   fs = []
   rx = []
   ry = []
   rz = []
   rot = [0,0,0]
   packetCount = []
   packetsDropped = 0
   imgCounter = 0
   portlist = []
   def __init__(self, parent = None):
      super(window, self).__init__(parent)
      uic.loadUi('main.ui',self)

      self.openGLWidget.initializeGL()
      self.openGLWidget.resizeGL(300,300)
      self.openGLWidget.paintGL = self.paintGL
      self.scene = pywavefront.Wavefront("payload.obj",collect_faces=True)
      self.scene.parse()
      GLUT.glutInit(sys.argv)
      scene_box = (self.scene.vertices[0], self.scene.vertices[0])
      for vertex in self.scene.vertices:
         min_v = [min(scene_box[0][i], vertex[i]) for i in range(3)]
         max_v = [max(scene_box[1][i], vertex[i]) for i in range(3)]
         scene_box = (min_v, max_v)

      self.scene_trans    = [-(scene_box[1][i]+scene_box[0][i])/2 for i in range(3)]

      scaled_size    = 1
      scene_size     = [scene_box[1][i]-scene_box[0][i] for i in range(3)]
      max_scene_size = max(scene_size)
      self.scene_scale    = [scaled_size/max_scene_size for i in range(3)]   
      print(max_scene_size)



      self.csvFile = CSVFileManager()
      
      
      self.ser = SerialManager()
      
      self.GCSTime = 0
      self.GCSTimer = QTimer(self)
      self.GCSTimer.setInterval(500)
      self.GCSTimer.setTimerType(Qt.PreciseTimer)
      self.GCSTimer.timeout.connect(self.addGCSTimer,type=Qt.DirectConnection)
      

      self.ser.telemetryProcessed.connect(self.csvFile.writeLine)
      self.ser.telemetryProcessed.connect(self.handlePlot)
      self.ser.telemetryProcessed.connect(self.updateGL)
      self.ser.missionTimeUpdated.connect(self.missionTime.setText)
      self.ser.imageProcessed.connect(self.displayImage)
    
      self.ser.comListUpdated.connect(self.updateComList, type=Qt.DirectConnection)
      self.ser.serP.packetDropped.connect(lambda: self.packetsDropped +1, type=Qt.DirectConnection)
      self.actionToggle_Serial_Settings_Dock.triggered.connect(self.toggleSerialSettingsVisible)
      self.actionClear_Plots.triggered.connect(self.clearPlots)
      self.connectButton.released.connect(self.ser.toggleConnection)
      self.connectButton.released.connect(self.GCSTimer.start)

      self.setWindowTitle("UAH Ground Station")
      self.comboBox_portList.textActivated.connect(self.ser.serP.serial.setPort)

      self.comboBox_baud.addItems(["115200","9600"])
      self.comboBox_baud.textActivated.connect(self.ser.serP.serial.setBaud)
      ##
      self.pen = pg.mkPen(color=(255, 0, 0), width=4)
      
      self.ser.serP.tx.connect(lambda: self.TX.setChecked(True),type = Qt.DirectConnection)
      self.ser.serP.rx.connect(lambda: self.RX.setChecked(True),type = Qt.DirectConnection)
      
      self.txTimer = QTimer(self)
      self.txTimer.setInterval(300)
      self.txTimer.setTimerType(Qt.PreciseTimer)
      self.txTimer.timeout.connect(lambda: self.TX.setChecked(False),type = Qt.DirectConnection)
      self.txTimer.start()

      self.rxTimer = QTimer(self)
      self.rxTimer.setInterval(180)
      self.rxTimer.setTimerType(Qt.PreciseTimer)
      self.rxTimer.timeout.connect(lambda:self.RX.setChecked(False),type = Qt.DirectConnection)
      self.rxTimer.start()

      self.graphWidget = pg.PlotWidget()
      self.graphWidget.setBackground('w')
      self.graphWidget.setTitle("Flight State",size = '20pt')
      styles = {'color':'k', 'font-size':'14px'}
      self.graphWidget.setLabel('left',"State", **styles)
      self.graphWidget.setLabel('bottom',"Time (s)", **styles)
      self.graphWidget.addLegend()
      self.graphWidget.showGrid(x=True,y=True)
      self.fs_data = self.graphWidget.plot([0],[0],name="Flight State",pen=self.pen)
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
      #self.tab_3.layout().replaceWidget(self.widget_4,  self.graphWidget4)


      # Send commands based on button presses
      self.ButtonManualSend.released.connect(self.manualSendCommand,type=Qt.DirectConnection)
      self.pushButton_resend1.released.connect(lambda: self.ser.serP.writeData(bytes("I1",'utf-8')))
      self.pushButton_resend2.released.connect(lambda: self.ser.serP.writeData(bytes("I2",'utf-8')))
      self.pushButton_resend3.released.connect(lambda: self.ser.serP.writeData(bytes("I3",'utf-8')))

      print("Main Thread: " + str(int(QThread.currentThreadId())))

      if(isSimulateSerial):
          self.p = QProcess()
          self.p.start("C:\\Users\\quaz9\\AppData\\Local\\Programs\\Python\\Python39\\python",["SimulateSerial.py"])
          #self.p.waitForFinished(-1)  

      """
      self.ser.serP.numImagesReceived = 2
      imageFile = QFile("image2.png")
      imageFile.open(QIODevice.ReadOnly)
      ba = imageFile.readAll()
      imageFile.close()
      
      self.ser.serP.receiveImage(ba)
      """

   @pyqtSlot()
   def addGCSTimer(self):
      self.GCSTime = self.GCSTime +0.5
      self.GCSTimeLabel.setText("GCS Time: T+ " + str(self.GCSTime))
   @pyqtSlot()
   def manualSendCommand(self):
      if(self.ser.serP.device):
         self.ser.serP.writeData(bytes(self.commandEdit.text(),"utf-8"))

   @pyqtSlot(bool)
   def toggleSerialSettingsVisible(self,stuff):
      self.dockSerialSettings.setVisible(not self.dockSerialSettings.isVisible())


   @pyqtSlot(list)
   def handlePlot(self,telList):
      
      self.packetCount.append(telList[0])
      self.Packets_received_label.setText("Packets Received: " + str(int(self.packetCount[-1])))
      self.Packets_lost_label.setText("Packets Dropped: " + str(int(self.packetsDropped)))
      self.times.append(telList[1])
      self.fs.append(telList[2])
      self.alt.append(telList[3])
      
      self.ax.append(telList[4])
      self.ay.append(telList[5])
      self.az.append(telList[6])

      self.rx.append(telList[7])
      self.ry.append(telList[8])
      self.rz.append(telList[9])
      

      self.fs_data.setData(self.times,self.fs)
      self.alt_data.setData(self.times,self.alt)

      self.ax_data.setData(self.times,self.ax)
      self.ay_data.setData(self.times,self.ay)
      self.az_data.setData(self.times,self.az)

      #self.rx_data.setData(self.times,self.rx)
      #self.ry_data.setData(self.times,self.ry)
      #self.rz_data.setData(self.times,self.rz)



      


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


   @pyqtSlot(list)
   def updateGL(self,telList):
      self.rot = telList[7:10]
      self.openGLWidget.update()



   def paintGL(self):
        


        glLightfv(GL_LIGHT0,GL_AMBIENT,(1.0,1.0,1.0,1.0),0)
        glLightfv(GL_LIGHT0,GL_DIFFUSE,(1.0,1.0,1.0,1.0),0)
        glLightfv(GL_LIGHT0,GL_SPECULAR,(1.0,1.0,1.0,1.0),0)
        glLightfv(GL_LIGHT0,GL_POSITION,(3,4,3,1),0)

        glClearColor(1.0,1.0,1.0,1)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluPerspective(45, int(self.openGLWidget.width())/float(self.openGLWidget.height()), 0.05, 50.0)
        glTranslatef(0,0, -7)
        glRotatef(45,1,0,0)
        glRotatef(45,0,1,0)
        
        glBegin(GL_LINES)
        glColor3f(1.0,0,0)
        glVertex3f(-10.0,0.0,0.0)
        glVertex3f(10.0,0.0,0.0)
        
        glColor3f(0.0,1.0,0.0)
        glVertex3f(0.0,-10.0,0.0)
        glVertex3f(0.0,10.0,0.0)

        glColor3f(0.0,0.0,1.0)
        glVertex3f(0.0,0.0,-10.0)
        glVertex3f(0.0,0.0,10.0)

        glEnd()

        glColor3f(0.0,0.0,0.0)
        glRasterPos3f(-4.2,0,-0.5)
        ang = math.sqrt(self.rot[0]*self.rot[0]+self.rot[2]*self.rot[2])
        errStr = "Leveling Angle: " + str(round(ang,1)) + " deg." 
        for c in errStr:
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,ord(c))


        glColor3f(1.0,0,0)
        glRasterPos3f(4, 0,0.5)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,ord('X'))
        
        glColor3f(0,1.0,0)
        glRasterPos3f(0.5, -4,0.5)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,ord('Y'))

        glColor3f(0,0,1.0)
        glRasterPos3f(0.1, 0.1,3)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,ord('Z'))

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glPushMatrix()
        #glScalef(*self.scene_scale)
        glTranslatef(*self.scene_trans)
        glRotatef(self.rot[0],1,0,0)
        glRotatef(self.rot[1],0,1,0)
        glRotatef(self.rot[2],0,0,1)

        """ glColor3f(0.0,0.0,0.0)
        for mesh in self.scene.mesh_list:
           glBegin(GL_LINES)
           for face in mesh.faces:
                 for vertex_i in face:
                    glVertex3f(*self.scene.vertices[vertex_i])
           glEnd()"""
        
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glShadeModel(GL_SMOOTH)
        count = 0
        """for mesh in self.scene.mesh_list:
           glBegin(GL_TRIANGLES)
           if(count<1):
               glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,(0.1,0.1,1.0))
               glMaterialfv(GL_FRONT,GL_SPECULAR,(1.0,1.0,1.0))
               glMaterialfv(GL_FRONT,GL_SHININESS,10)
           else:
               glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,(0.1,0.1,0.1))
               glMaterialfv(GL_FRONT,GL_SPECULAR,(1.0,1.0,1.0))
               glMaterialfv(GL_FRONT,GL_SHININESS,10)
            
           count = count+1
           for face in mesh.faces:

                 for vertex_i in face:
                    glVertex3f(*self.scene.vertices[vertex_i])
           glEnd()"""
        glColor3f(1.0,1.0,1.0)
        meshes = Wavefront("payload.obj")
        visualization.draw(meshes)

        glDisable(GL_LIGHTING)
        glDisable(GL_LIGHT0)   

        glBegin(GL_LINES)
        glColor3f(1.0,0,1.0)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(1.0,0.0,0.0)

       
        glVertex3f(0.0,0.0,0.0)
        if(np.linalg.norm(np.array(self.rot),1) > 1.0):
            orient = [self.rot[0] / np.linalg.norm(np.array(self.rot),1),self.rot[1] / np.linalg.norm(np.array(self.rot),1),self.rot[2] / np.linalg.norm(np.array(self.rot),1)]
        else:
            orient = [0,0,0]
        #print(orient)
        s = 5
        glVertex3f(0.0,s,0.0)

        glColor3f(0.0,0.0,0.0)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(s,0.0,0.0)
        glEnd()
        
        glColor3f(1.0,0.0,1.0)
        glRasterPos3f(0.1,2,0.1)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10,ord('O'))


        glColor3f(0.0,0.0,0.0)
        glRasterPos3f(2,0.0,0.3)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10,ord('P'))
        glPopMatrix()
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)




        glFlush()
        #time.sleep(0.1)

   @pyqtSlot(bool)
   def clearPlots(self,stuff):
      self.times.clear()
      self.fs.clear()
      self.alt.clear()
      
      self.ax.clear()
      self.ay.clear()
      self.az.clear()

      self.rx.clear()
      self.ry.clear()
      self.rz.clear()
            
      self.fs_data.setData(self.times,self.fs)
      self.alt_data.setData(self.times,self.alt)

      self.ax_data.setData(self.times,self.ax)
      self.ay_data.setData(self.times,self.ay)
      self.az_data.setData(self.times,self.az)

      print("Plots Clear")
      



def main():
   app = QApplication(sys.argv)


   ex = window()
   ex.show()
   sys.exit(app.exec_())




   
if __name__ == '__main__':
   main()
