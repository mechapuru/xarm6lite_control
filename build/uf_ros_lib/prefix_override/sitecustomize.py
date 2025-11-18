import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/paddy/rrc/xarm_ws/src/xarm_ros2/install/uf_ros_lib'
