import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import TimerAction, IncludeLaunchDescription, AppendEnvironmentVariable, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle'
    urdf_name='URDF1.urdf.xacro'
    world_sdf_file = 'fws_robot_world.sdf'
     
   

    #Set paths to different files and packages
    urdf=os.path.join(get_package_share_directory(package_name),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'
    path_to_world_sdf = os.path.join( get_package_share_directory(package_name),'world', world_sdf_file)
    resource_path = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')

    #Gazebo models path
    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    #Rviz config file
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    #URDF Parser
    doc = xacro.process_file(urdf)

    #Robot description argument
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}

    #Set the use_sim_time parameter globally
    #set_sim_time = SetLaunchConfiguration(name='use_sim_time', value='True')

    mapper_params_online_async_file = os.path.join(
       pkg_share, 'config', 'mapper_params_online_async.yaml'
    )
   
    # Nodes
    #Robot_state_publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description,
                   ],
        
        
    )

    #Joint_state_publisher Node
    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    #Rviz Node; Launches RVIZ
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
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
    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive"],
        parameters=[{'use_sim_time': True}]
    )

    #Joint state broadcaster Node; Initializes Joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
    )
    

    # Bridge Node: Initializes lidar scan topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/scan2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/clock@rosgraph_msgs/msg/clock@gz.msgs.clock',
                   ],
        output='screen',
    )
     
    #3 second delayed velocity spawner
    delayed_velocity= TimerAction(
        period=3.0,  # Delay in seconds
        actions=[diff_spawner],
    )

    #2.5 second delayed joint state broadcaster spawner
    delayed_joint= TimerAction(
        period=2.5,  # Delay in seconds
        actions=[joint_broad_spawner],
    )
    

    diff_publisher= Node(
        executable='diff_publisher.py',
        name='diff_publisher',
        output='screen',
    )

    delayed_publisher= TimerAction(
        period=4.0,  #4 Second Delay
        actions=[diff_publisher],
    )
    
    bridge_params=os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')

    ros_gz_bridge2= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    online_async_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': mapper_params_online_async_file,
            'use_sim_time': 'true'
        }.items()
    )
        

    return LaunchDescription([
        #SetParameter(name='use_sim_time', value=True),  # Global setting
        robot_state_publisher_node,
        ros_gz_bridge2,
        set_env_vars_resources2,
        joint_state_publisher_node,
        launch_rviz_node,
        gz_sim,
        spawn_entity,
        delayed_velocity,
        delayed_joint,
        bridge,
        delayed_publisher,   
        online_async_slam,  
    ])
