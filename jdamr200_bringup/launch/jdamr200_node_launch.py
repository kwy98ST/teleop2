import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    jdamr200_urdf = os.path.join(
        get_package_share_directory('jdamr200_description'),
        'urdf',
        'jdamr200.urdf')
    with open(jdamr200_urdf, 'r') as file:
        jdamr200_desc = file.read()

    ld14lidar_launch_file_dir = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'),
        'launch'
    )
    
    robot_param = {'robot_description': jdamr200_desc}
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
     

        Node(
            package='jdamr200_node',
            executable='jdamr200_node',
            output='screen',
        ),
    ])