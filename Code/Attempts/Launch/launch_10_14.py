import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'urdf/URDF1.urdf'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('turtle_on_land_demo'),
      urdf_file_name)

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "turtle_on_land_demo"])
  ])
