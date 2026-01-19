import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_ros_gazebo')
    
    # 1. Launch Simulation (Empty World + Bridge + Spawner)
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn.launch.py')
        )
    )

    # 2. Launch RL Driver Node
    # Wait 5 seconds for simulation to stabilize
    rl_driver_node = Node(
        package='go2_ros_gazebo',
        executable='go2_rl_driver',
        name='go2_rl_driver',
        output='screen'
    )
    
    delayed_rl_driver = TimerAction(
        period=5.0,
        actions=[rl_driver_node]
    )

    return LaunchDescription([
        spawn_launch,
        delayed_rl_driver
    ])
