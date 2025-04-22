import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tristan/ros2_ws/src/diff_drive_controller/install/diff_drive_controller'
