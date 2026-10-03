import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('living_map_gazebo')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'usar_sim.launch.py')))

    outside_network = Node(package='living_map_system', executable='outside_network_node', output='screen')
    command_post = Node(package='living_map_system', executable='command_post_node', output='screen')

    writer = TimerAction(period=8.0, actions=[
        Node(package='living_map_system', executable='writer_node', output='screen')])
    executor = TimerAction(period=8.0, actions=[
        Node(package='living_map_system', executable='executor_node', output='screen')])

    return LaunchDescription([
        sim_launch,
        outside_network,
        command_post,
        writer,
        executor,
    ])
