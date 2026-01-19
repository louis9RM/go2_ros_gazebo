import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ever/ros2_ws/src/go2_ros_gazebo/install/go2_ros_gazebo'
