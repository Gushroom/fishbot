import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/psf/Home/Desktop/ROS2/SLAM/fishbot/fishbot_description/install/fishbot_description'
