import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mikkel/git/Robot-Autonomy/workspace/ros2_ws1/install/my_turtlebot'
