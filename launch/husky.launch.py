##
# Based on clearpath_gz simulation.launch.py and gz_sim.launch.py
# 
import os


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


my_package_name = 'prs49_ros366_cpp'

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='track1',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='husky', description='Robot namespace'),
]
# Add arguments to specify initial pose of the robot
for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))




def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros366_gz = get_package_share_directory('ros366_gz')
    colour_detector = IncludeLaunchDescription(
	XMLLaunchDescriptionSource([
	FindPackageShare(my_package_name), '/launch',
	'/colour_detector.launch.xml'])
	)
    
    launch_rviz = LaunchConfiguration("rviz")

    #Paths
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_ros366_gz, 'worlds'), ':',
            os.path.join(pkg_ros366_gz, 'models'), ':',
            os.path.join(pkg_clearpath_gz, 'worlds'),
            ':' + ':'.join(packages_paths)])



    # Gazebo Simulator

    gui_config = PathJoinSubstitution(
        [pkg_ros366_gz, 'config', 'gui.config'])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                         ' -r',
                         ' -v 4',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    # Spawning the robot into Gazebo
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world')),
            ('rviz', 'false'),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    # Launching RVIZ
    pkg_myPackage = FindPackageShare(my_package_name)


    config_rviz = PathJoinSubstitution(
        [pkg_myPackage, 'config', 'husky_rviz.config.rviz']
    )

    group_view_model = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', config_rviz],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen')
    ])


    # Clock bridge (enabled communication between ROS and gz)
    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                          '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                        ])
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    ld.add_action(clock_bridge)
    ld.add_action(group_view_model)
    ld.add_action(colour_detector)
    return ld
