import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    gazebo_share = get_package_share_directory('living_map_gazebo')
    desc_share = get_package_share_directory('living_map_description')
    world = os.path.join(gazebo_share, 'worlds', 'usar_building.world')

    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    def xacro_to_urdf(xacro_file, ns):
        return ExecuteProcess(
            cmd=['bash', '-c',
                 f'xacro {xacro_file} ns:={ns} > /tmp/{ns}.urdf'],
            output='screen',
        )

    writer_xacro = os.path.join(desc_share, 'urdf', 'writer_drone.xacro')
    executor_xacro = os.path.join(desc_share, 'urdf', 'executor_rover.xacro')

    gen_writer_urdf = xacro_to_urdf(writer_xacro, 'writer')
    gen_executor_urdf = xacro_to_urdf(executor_xacro, 'executor')

    spawn_writer = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 run gazebo_ros spawn_entity.py -entity writer_drone '
             '-file /tmp/writer.urdf -robot_namespace writer '
             '-x -8.5 -y -8.5 -z 1.5'],
        output='screen')])

    spawn_executor = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 run gazebo_ros spawn_entity.py -entity executor_rover '
             '-file /tmp/executor.urdf -robot_namespace executor '
             '-x -8.5 -y -8.0 -z 0.1'],
        output='screen')])

    return LaunchDescription([
        gazebo,
        gen_writer_urdf,
        gen_executor_urdf,
        spawn_writer,
        spawn_executor,
    ])
