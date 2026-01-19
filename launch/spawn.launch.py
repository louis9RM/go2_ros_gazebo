import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "go2_ros_gazebo"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    declare_urdf_model = DeclareLaunchArgument(
        name='urdf_model', 
        default_value='go2_description.urdf', 
        description='URDF model to spawn'
    )
    
    urdf_file = LaunchConfiguration('urdf_model')
    robot_desc_path = PathJoinSubstitution([package_directory, "urdf", urdf_file])
    
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path])}]
    )


# ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
        # Reloj de la simulación
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        
        # Sensores (Mapeados según tus etiquetas <topic>)
        "/go2/radar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/go2/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        "/go2/camera@sensor_msgs/msg/Image[ignition.msgs.Image",

        # Articulaciones - Pata Delantera Izquierda (FL)
        "/go2/joint/FL_hip_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/FL_thigh_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/FL_calf_joint@std_msgs/msg/Float64@gz.msgs.Double",

        # Articulaciones - Pata Delantera Derecha (FR)
        "/go2/joint/FR_hip_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/FR_thigh_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/FR_calf_joint@std_msgs/msg/Float64@gz.msgs.Double",

        # Articulaciones - Pata Trasera Izquierda (RL)
        "/go2/joint/RL_hip_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/RL_thigh_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/RL_calf_joint@std_msgs/msg/Float64@gz.msgs.Double",

        # Articulaciones - Pata Trasera Derecha (RR)
        "/go2/joint/RR_hip_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/RR_thigh_joint@std_msgs/msg/Float64@gz.msgs.Double",
        "/go2/joint/RR_calf_joint@std_msgs/msg/Float64@gz.msgs.Double",
    ],
    output="screen",
    )



    # Spawn the Robot #
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0",
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0",
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5",
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="go2_spawn",
        arguments=[
            "-name", "go2",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )





    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            declare_urdf_model,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            robot_state_publisher_node,
            gz_spawn_entity,
            ign_bridge,
        ]
    )
