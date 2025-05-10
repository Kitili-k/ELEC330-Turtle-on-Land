import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle'
    urdf_name='URDF1.urdf.xacro'
    urdf=os.path.join(get_package_share_directory(package_name),'urdf',urdf_name)
    rviz_config_file='rviz/rviz_basic_settings.rviz'
    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')


    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Delay activation of SLAM toolbox to ensure all topics are available
    delayed_slam_toolbox = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[slam_toolbox_node]
    )


    # Set paths to different files
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)

    doc = xacro.process_file(urdf, mappings={'use_sim_time' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}

    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='fws_robot_world',
                          description='Gz sim World'),
           ]
    )

    # Launch RViz2
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    
    #Velocity
    velocity= Node(
        package='turtle',
        executable='publisher.py',
        name='vel_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r',]
                    ),
                    ('use_sim_time', 'true')
                ]
             )

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
    resource_path = get_package_share_directory('turtle')
    

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(resource_path, 'worlds'), ':' +
            str(Path(resource_path).parent.resolve())
            ]
        )

    Velcoity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_vel"],
        parameters=[{'use_sim_time': True}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        velocity,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch_rviz_node,
        gz_sim,
        spawn_entity,
        Velcoity_spawner,
        joint_broad_spawner,
        bridge,
        delayed_slam_toolbox,
        
    ])
