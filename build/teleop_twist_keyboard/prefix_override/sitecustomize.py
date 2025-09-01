import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/my_robot_ws_copy/install/teleop_twist_keyboard'
