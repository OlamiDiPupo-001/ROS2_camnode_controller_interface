import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/olamidipupo/ros2_ws/src/install/camera_click_teleop'
