#! /usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QLineEdit,  QPushButton, QVBoxLayout
 
########################################################################
class MainWindow(QMainWindow):
	
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Elements de l'IHM
        self.setMinimumSize(300, 300)    
        self.setWindowTitle("ROS2 IHM") 
         
        self.btn = QPushButton(self)
        self.btn.setText('SEND ROS MESSAGE')
        self.btn.clicked.connect(self.onPushSend)  # Action associée à l'appui sur le bouton ( signal )

        # Disposition des Elements
        widget = QWidget()
        self.setCentralWidget(widget)
        vb = QVBoxLayout(widget)

        vb.addStretch()
        vb.addWidget(self.btn)
        vb.addStretch()
        
        # ROS2
        rclpy.init(args=None) 
        self.node = Node('py_ihm_node')
        self.publisher_ = self.node.create_publisher(String, 'pyqt_topic_send', 10)
        self.i = 0
        
        self.subscription = self.node.create_subscription(
            String,
            'pyqt_topic_rec',
            self.listener_callback,
            10)
           
        self.subscription  # prevent unused variable warning
              
        self.timer = QTimer()
        self.timer.timeout.connect(self.onTimerTick)   
        self.timer.start(5)      
  
    def onPushSend(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        print('Publishing: "%s"' % msg.data)
        self.i += 1
        
    def listener_callback(self, msg):
        print('I heard: "%s"' % msg.data)    
        
    def onTimerTick(self):  
        rclpy.spin_once(self.node,executor=None, timeout_sec=0.01)
               
########################################################################
def main(args=None):
 
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()
########################################################################
   
