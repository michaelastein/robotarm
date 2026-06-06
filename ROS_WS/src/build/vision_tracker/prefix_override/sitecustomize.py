import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/micha/RobotArm/ROS_WS/src/install/vision_tracker'
