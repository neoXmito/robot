#! /usr/bin/python3
#
# colcon build --packages-select pyqt_rcv_cam_ihm_ros2
# . install/setup.bash
# ros2 run pyqt_rcv_cam_ihm_ros2 rcv_cam_ihm

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QGraphicsScene, QGraphicsView, QGridLayout
)

########################################################################
# Thread ROS2 
########################################################################
class ROS2Thread(QtCore.QThread):
    image_received = QtCore.pyqtSignal(object)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node('PC_HOST_Node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Souscription au topic src_frame
        self.subscription = self.node.create_subscription(
            Image,
            '/camera/src_frame',
            self.callback,
            qos_profile
        )

        self.running = True

    def callback(self, msg):
        # Signal vers le thread principal (IHM)
        self.image_received.emit(msg)

    def run(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        self.running = False
        self.node.destroy_node()
        rclpy.shutdown()

########################################################################
# Fenêtre principale PyQt
########################################################################
class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.setMinimumSize(800, 800)
        self.setWindowTitle("ROS2 RECEIVE CAM IHM")

        # Disposition des éléments 
        widget = QWidget()
        self.setCentralWidget(widget)

        self.graphicsScene = QGraphicsScene()
        self.graphicsView = QGraphicsView()
        self.graphicsScene.setBackgroundBrush(QColor(0, 0, 0))
        self.graphicsView.setScene(self.graphicsScene)
        self.graphicsView.setAlignment(Qt.AlignCenter)

        layout = QGridLayout(widget)
        layout.addWidget(self.graphicsView, 0, 0)

        # Lancer ROS2 dans un thread 
        self.ros_thread = ROS2Thread()
        self.ros_thread.image_received.connect(self.img_callback)
        self.ros_thread.start()

        self.pix_item = None

    def closeEvent(self, event):
        if self.ros_thread.isRunning():
            self.ros_thread.stop()
            self.ros_thread.wait()
        event.accept()

    ####################################################################
    def img_callback(self, img):
        try:
            # Conversion ROS2 --> QImage
            image = QImage(img.data, img.width, img.height, QImage.Format_BGR888)

            # MAJ
            pixmap = QPixmap.fromImage(image)
            if self.pix_item is None:
                self.pix_item = self.graphicsScene.addPixmap(pixmap)
            else:
                self.pix_item.setPixmap(pixmap)

        except Exception as e:
            print(f"[Erreur conversion image] {e}")

########################################################################
def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

########################################################################
if __name__ == '__main__':
    main()
########################################################################
