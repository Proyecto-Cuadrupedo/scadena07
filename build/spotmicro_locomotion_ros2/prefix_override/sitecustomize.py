import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sacad/spotmicro_ws/install/spotmicro_locomotion_ros2'
