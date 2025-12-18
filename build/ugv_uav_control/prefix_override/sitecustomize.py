import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shlok-mehndiratta/Desktop/Project_ws/install/ugv_uav_control'
