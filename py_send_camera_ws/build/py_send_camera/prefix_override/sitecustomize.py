import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mito/proj_robot/py_send_camera_ws/install/py_send_camera'
