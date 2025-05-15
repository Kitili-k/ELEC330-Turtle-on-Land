import os
import xacro
from launch import LaunchDescription
from pathlib import Path
from launch.actions import TimerAction, IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    """
    Launch file for the turtle robot simulation.
    Launches Gazebo, SLAM, navigation and other required nodes.
    """
    # Package and file paths
    package_name = 'turtle'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # File paths
    urdf_name = 'URDF1.urdf.xacro'
    world_sdf_file = 'world3.sdf'
    rviz_config_file = 'rviz/rviz_basic_settings.rviz'
    
    # Set up paths
    urdf = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_name)
    rviz_config_path = os.path.join(pkg_share, rviz_config_file)
    path_to_world_sdf = os.path.join(get_package_share_directory(package_name), 'world', world_sdf_file)
    resource_path = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    mapper_params_online_async_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    log_level = LaunchConfiguration('log_level', default='info')

    # Set up Gazebo model path
    gazebo_models_path = os.path.join(pkg_share, 'meshes')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Parse URDF
    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {"robot_description": robot_desc}

    # Create launch configuration parameters
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description, 
            {'use_sim_time': use_sim_time},
            {'publish_frequency': 50.0},  # increase publish frequency
            {'frame_prefix': ''},  # ensure no extra prefix
        ],
    )

    # Joint State Publisher Node - ensure all joint states are published
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'source_list': ['/joint_states']},  # specify using Gazebo's joint_states
            {'rate': 50}  # increase publish frequency, ensure TF tree update frequency is high enough
        ],
        output='screen',
    )

    # RViz Node
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 {path_to_world_sdf}"}.items(),
    )

    # Spawn Robot Entity
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

    # Set Gazebo Resource Path
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(resource_path)).parent.resolve())
    )

    # ROS-Gazebo Bridge Node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # sensor data - put high priority ones first
            '/scan2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera@vision_msgs/msg/Detection2DArray@gz.msgs.AnnotatedAxisAligned2DBox_V',
            # robot control and state
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/cmd_vel_stamped@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',  # re-enable TF bridge
            # system clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_position_controller@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory'
        ],
        output='screen',
        
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    # static TF publisher - ensure base_footprint to base_link transform
    static_base_footprint_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # static TF publisher - connect Sensor_link to base_link
    static_sensor_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_sensor_link',
        arguments=['0.0975', '0', '0.005', '0', '0', '0', 'base_link', 'Sensor_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # SLAM Toolbox Online Async Launch
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
    
    # Bounding Box Controller Node
    bounding_box_controller = Node(
        package='turtle',
        executable='BoundingBox_callback.py',
        name='bounding_box_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Nav2 Navigation Launch
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_respawn': use_respawn
        }.items()
    )

    # Twist Mux Node    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True},{'use_stamped':False}],
            output='screen',
    )

    # Auto Explore Node
    auto_explore_node = Node(
        package='turtle',
        executable='auto_explore.py',
        name='auto_explore',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        prefix='python3 ' + os.path.join(pkg_share, 'scripts', ''),
    )

    # Return Launch Description
    return LaunchDescription([
        # declare parameters
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        
        # start basic nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        set_env_vars_resources2,
        twist_mux,
        
        # start simulation and visualization
        gz_sim,
        
        # wait for Gazebo to fully start
        TimerAction(
            period=1.0,
            actions=[spawn_entity]
        ),
        bridge,

        # wait for bridge node to start before starting TF publisher
        static_base_footprint_link,
        static_sensor_link,
        
        # wait for TF publisher to start before starting SLAM
        TimerAction(
            period=2.0,
            actions=[online_async_slam]
        ),
        
        # start visualization and control
        launch_rviz_node,
        
        # start bounding box controller after all systems are running
        TimerAction(
            period=5.0,
            actions=[bounding_box_controller]
        ),
        
        # disable Nav2 navigation - comment out temporarily
        TimerAction(
            period=5.0,
            actions=[nav2_navigation]
        ),

        TimerAction(
            period=6.0,
            actions=[auto_explore_node]
        ),
    ])
