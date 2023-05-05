import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_path = os.path.join(get_package_share_directory('auto_park2'))
    xacro_file = os.path.join(package_path,'description','prius.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    sim_params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[sim_params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false',description='use sim time'),node_robot_state_publisher])
