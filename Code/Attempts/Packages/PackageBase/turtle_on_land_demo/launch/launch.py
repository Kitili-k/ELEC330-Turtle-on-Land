#Essential Libraries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = 'URDF1.urdf'

    # Specify the full path to the URDF file
    path_to_urdf = os.path.join(
        get_package_share_directory('turtle_on_land_demo'),
        'urdf',
        urdf_file)

    # Set environment variable for GZ_SIM_RESOURCE_PATH
    resource_path = get_package_share_directory('turtle_on_land_demo')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)
        }]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    # Spawn the robot in Gazebo using the EntityFactory service equivalent
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",  # 'create' executable acts like the service to spawn a model in Gazebo
        arguments=[
            "-name", "robot1",  # Name of the robot
            "-file", path_to_urdf,  # Use the absolute path to your URDF file
            "-x", "0", "-y", "0", "-z", "1.4"  # Initial position of the robot in the world
        ],
        output="screen",
    )

    # Add the ros_gz_bridge node to bridge the joint states and commands
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Bridge joint states from ROS 2 to Gazebo transport
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Bridge velocity commands if controlling with velocity
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist'
        ],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        # Set the environment variable for GZ_SIM_RESOURCE_PATH
        set_gz_sim_resource_path,

        # Start robot state publisher, Gazebo simulator, and spawn the robot
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge  # Add the ros_gz_bridge node
    ])

