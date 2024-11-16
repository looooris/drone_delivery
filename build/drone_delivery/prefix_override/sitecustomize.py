import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/loris/Documents/Projects/ROS2/drone_delivery_main/install/drone_delivery'
