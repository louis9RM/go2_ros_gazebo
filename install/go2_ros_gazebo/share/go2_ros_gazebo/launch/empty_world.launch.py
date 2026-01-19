import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "go2_ros_gazebo"
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = os.path.join(get_package_prefix(package_description), "share")
    robot_meshes_path = os.path.join(package_directory, "dae")
    
    # Append to the GZ_SIM_RESOURCE_PATH environment variable
    # This allows Gazebo to find the mesh files
    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir_path + ":" + robot_meshes_path
    )

    # Load Empty World SDF from Gazebo Sim Package #
    world_file = os.path.join(package_directory, "worlds", "demo_world.sdf")
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=["-r ", world_file],
                                              description="SDF World File")
    
    # Declare Gazebo Sim Launch #
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_sim_pkg, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": world_config}.items(),
    )

    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            gz_resource_path,
            declare_world_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
        ]
    )
