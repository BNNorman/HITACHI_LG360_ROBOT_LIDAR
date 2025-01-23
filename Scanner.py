# importing Qt widgets
from PyQt5.QtWidgets import *

# importing system
import sys

# importing numpy as np
import math
import numpy as np

# importing pyqtgraph as pg
import pyqtgraph as pg
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import HITACHI_LDS360_LIDAR as Lidar

import traceback

class Window(QMainWindow):

    dX=np.array([0]*360,dtype=np.float32)
    dY=np.array([0]*360,dtype=np.float32)
    iX=np.array([0]*360,dtype=np.float32)
    iY=np.array([0]*360,dtype=np.float32)

    lidar=Lidar.HITACHI_LDS360()
    
    def __init__(self):
        super().__init__()

        # setting title
        self.setWindowTitle("PyQtGraph")

        # setting geometry
        self.setGeometry(100, 100, 600, 500)

        # icon
        icon = QIcon("skin.png")

        # setting icon to the window
        self.setWindowIcon(icon)

        # calling method
        self.UiComponents()

        # showing all the widgets
        self.show()
        
        self.timer = QTimer(self) # qtcore
        self.timer.setInterval(20) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.updateScatterDiagrams)
        
        self.max_intensity=1000 # used for scaling into 0..1000 range
        self.max_distance=0
        
        self.lidar.setCallback(self.onNewPass) 
        self.lidar.start()
        
        self.newData=False # callback uses this flag
        

    def __del__(self):
        print("Window() closed")
        self.lidar.stop()

    def setDistancePoint(self,angle,value):
        self.dX[angle]=value * math.cos(math.radians(angle))
        self.dY[angle]=-1*value * math.sin(math.radians(angle))

    def setIntensityPoint(self,angle,value):
        if value>1000:
            value=1000
        self.iX[angle]=value * math.cos(math.radians(angle))
        self.iY[angle]=-1*value * math.sin(math.radians(angle))

    def onNewPass(self):
        # callback - just set a flag
        self.newData=True
        
    def updateScatterDiagrams(self):
        """
        The driver stores 360 sequencial values for distance and intensity
        """
        if not self.newData:
            return
           
        #print("Updating scatter diagrams")
        
        for angle in range(360):
            self.setDistancePoint(angle,self.lidar.distanceView[angle])
            self.setIntensityPoint(angle,self.lidar.intensityView[angle])
        
        self.scatterD.setData(self.dX,self.dY)
#        self.scatterI.setData(self.iX,self.iY)
        self.newData=False
        
    # method for components
    def UiComponents(self):

        # creating a widget object
        widget = QWidget()

        # creating a label
        label = QLabel("Geeksforgeeks Scatter Plot")

        # making label do word wrap
        label.setWordWrap(True)

        # creating a plot window
        plot = pg.plot()
        

        # creating a scatter plot item
        # of size = 10
        # using brush to enlarge the of green color
        self.scatterD = pg.ScatterPlotItem(size=10, pen="r") #, brush=pg.mkBrush(30, 255, 0, 0))
        self.scatterI = pg.ScatterPlotItem(size=5, pen='g') #brush=pg.mkBrush(30, 0,255,0))
        
        # initialise
        # works but PyQtGraph goes on to auto-scale
        
        dx=[-3500,3500,3500,-3500]
        dy=[3500,3500,-3500,-3500]
        ix=[-100,100,100,-100]
        iy=[100,100,-100,-100]
        
        # adding spots to the scatter plot
        self.scatterD.addPoints(dx,dy)
        self.scatterI.addPoints(ix,iy)

        # add items to plot window
        # adding scatter plot items to the plot window
        # then turn off auto-ranging
        plot.addItem(self.scatterD)
        plot.addItem(self.scatterI)
        plot.autoRange(False)

        # Creating a grid layout
        layout = QGridLayout()

        # minimum width value of the label
        label.setMinimumWidth(130)

        # setting this layout to the widget
        widget.setLayout(layout)

        # adding label in the layout
        layout.addWidget(label, 1, 0)

        # plot window goes on right side, spanning 3 rows
        layout.addWidget(plot, 0, 1, 3, 1)

        # setting this widget as central widget of the main window
        self.setCentralWidget(widget)


        # setting text to the value
        label.setText("Data")


try:
        
    # create pyqt5 app
    App = QApplication(sys.argv)

    # create the instance of our Window
    window = Window()

    # start the app
    sys.exit(App.exec())
    
    
    
except Exception as e:
    print(f"EXCEPTION: {e}")
    traceback.print_exc()
        
