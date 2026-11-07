import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('navapex_ros2'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    turtlebot = Path(get_package_share_directory('turtlebot3_gazebo'))
    use_sim_time = {'use_sim_time': True}
    robot_description = (turtlebot / 'urdf/turtlebot3_burger.urdf').read_text()
    costmaps = str(share / 'config/costmaps.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('config_dir', default_value=str(share / 'config')),
        DeclareLaunchArgument('log_directory', default_value=str(Path.cwd() / 'log')),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('NAVAPEX_CONFIG_DIR', LaunchConfiguration('config_dir')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(turtlebot / 'models') + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gzserver.launch.py')),
                                 launch_arguments={'world': str(share / 'worlds/navapex.world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[use_sim_time, {'robot_description': robot_description}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', prefix=['/usr/bin/python3'], arguments=[
            '-entity', 'burger', '-file', str(turtlebot / 'models/turtlebot3_burger/model.sdf'),
            '-x', '0', '-y', '0', '-z', '0.01'], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'], parameters=[use_sim_time]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[use_sim_time, {'yaml_filename': str(share / 'maps/navapex.yaml')}], output='screen'),
        Node(package='navapex_ros2', executable='navapex_costmap_node', name='global_costmap',
             namespace='global_costmap', parameters=[costmaps], output='screen'),
        Node(package='navapex_ros2', executable='navapex_costmap_node', name='local_costmap',
             namespace='local_costmap', parameters=[costmaps], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle_manager',
             parameters=[use_sim_time, {'autostart': True, 'node_names': ['map_server']}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
             namespace='global_costmap', parameters=[use_sim_time, {
                 'autostart': True, 'node_names': ['global_costmap']}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
             namespace='local_costmap', parameters=[use_sim_time, {
                 'autostart': True, 'node_names': ['local_costmap']}], output='screen'),
        Node(package='navapex_ros2', executable='navapex_node', name='navapex_planner',
             parameters=[str(share / 'config/navapex.yaml'), {
                 'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                 'log_directory': LaunchConfiguration('log_directory')}], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/navapex.rviz')],
             parameters=[use_sim_time], condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
