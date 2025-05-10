import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import  TimerAction, IncludeLaunchDescription, SetLaunchConfiguration, ExecuteProcess, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle_on_land_demo'
    urdf_name='URDF1.urdf.xacro'
    world_sdf_file = 'world.sdf'

    #Set paths to different files and packages
    urdf=os.path.join(get_package_share_directory(package_name),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'
    path_to_world_sdf = os.path.join( get_package_share_directory(package_name),'world', world_sdf_file)
    resource_path = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    #Gazebo models path
    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path


    #Rviz config file
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    #URDF Parser
    doc = xacro.process_file(urdf, mappings={'use_sim' : 'true'})

    #Set the use_sim_time parameter globally
    set_sim_time = SetLaunchConfiguration('use_sim_time', 'true')
   

    #Robot description argument
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}
   
    # Nodes
    #Robot_state_publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    #Joint_state_publisher Node
    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    #Rviz Node; Launches RVIZ
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_path],
        parameters=[
        {'use_sim_time': True},  # Ensure simulated time is used
        {'tf_buffer_size': 120.0},  # Increase transform buffer size
     ]
    )

    # Gazebo Node: Launches Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r -v 4 {path_to_world_sdf}"}.items(),
    )

    #Spawns entity Node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'turtle',
                   '-allow_renaming', 'false'],
    )

    # Set environment variable for GZ_SIM_RESOURCE_PATH
    

    #Gazebo resources
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(resource_path)).parent.resolve()))

    #Joint Vel Controller Node; Initiazlizes joint velocity control
    Velcoity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_vel"],
        parameters=[{'use_sim_time': True}],
    )

    #Joint state broadcaster Node; Initializes Joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
        
    )
    

    # Bridge Node: Initializes lidar scan topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/scan2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/scan/points','/input_cloud')
        ],
    )
     
     
    #3 second delayed velocity spawner
    delayed_velocity= TimerAction(
        period=3.0,  # Delay in seconds
        actions=[Velcoity_spawner],
    )

    #2.5 second delayed joint state broadcaster spawner
    delayed_joint= TimerAction(
        period=2.5,  # Delay in seconds
        actions=[joint_broad_spawner],
    )
    

    velocity_joint_publisher= Node(
        package=package_name,
        executable='publisher',
        name='vel_publisher',
        output='screen',
    )

    delayed_publisher= TimerAction(
        period=4.0,  #4 Second Delay
        actions=[velocity_joint_publisher],
    )

    force= ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers', '--activate joint_vel'],
            output='screen'
    )

    force2= ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers', '--activate joint_state_broadcaster'],
            output='screen'
    )

    lidar_slam=ExecuteProcess(
        cmd=['ros2', 'launch', 'lidarslam', 'lidarslam.launch.py'],
        output='screen',
    )

    

    return LaunchDescription([
        set_sim_time,
        set_env_vars_resources2,
        joint_state_publisher_node,
        robot_state_publisher_node,
        lidar_slam,
        #launch_rviz_node,
        gz_sim,
        spawn_entity,
        delayed_velocity,
        delayed_joint,
        bridge,
        force,
        force2,
        delayed_publisher,

    ])
