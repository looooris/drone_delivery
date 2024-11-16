import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/loris/ros2_ws/drone_delivery_main/install/drone_delivery'
