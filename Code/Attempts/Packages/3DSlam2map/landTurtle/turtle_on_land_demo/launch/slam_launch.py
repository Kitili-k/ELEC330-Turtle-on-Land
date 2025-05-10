import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('turtle_on_land_demo')

    slam_config_path = os.path.join(pkg_share, 'config', 'slam_config.yaml')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': use_sim_time},
            {'map_update_interval': 1.0},      
            {'transform_publish_period': 0.05}, 
            {'resolution': 0.05},              
            {'max_laser_range': 20.0},         
            {'minimum_time_interval': 0.2},   
        ]
    )

    return LaunchDescription([
        slam_node
    ])