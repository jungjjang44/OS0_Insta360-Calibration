import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/krri/OS0_Insta360-Calibration/sensor_ws/install/test_package'
