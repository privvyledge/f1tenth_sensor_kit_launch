from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    f1tenth_sensor_kit_launch_dir = get_package_share_directory('f1tenth_sensor_kit_launch')

    # Create the launch configuration variables
    lidar_config = LaunchConfiguration('lidar_config')
    launch_filter = LaunchConfiguration('launch_filter')

    # Launch arguments
    lidar_config_file = os.path.join(
            f1tenth_sensor_kit_launch_dir,
            'config/laserscan',
            '2d_lidar.param.yaml')
    laser_filter_config_file = os.path.join(
            f1tenth_sensor_kit_launch_dir,
            'config/laserscan',
            'laser_filter.param.yaml')

    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config_file,
                                     description='Path to the YDLIDAR parameters file to use.')
    laser_filter_la = DeclareLaunchArgument('laser_filter_config',
                                            default_value=lidar_config_file,
                                            description='Path to the YDLIDAR parameters file to use.')

    declare_launch_filter_cmd = DeclareLaunchArgument(
            'launch_filter',
            default_value='True',
            description='Whether to launch the LIDAR filter')

    # Create Launch Description
    ld = LaunchDescription([lidar_la, laser_filter_la, declare_launch_filter_cmd])

    # Setup nodes
    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                               executable='ydlidar_ros2_driver_node',
                               name='ydlidar_ros2_driver_node',
                               output='screen',
                               emulate_tty=True,
                               parameters=[lidar_config],
                               namespace='lidar')

    laserscan_filter_node = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace='lidar',
            parameters=[laser_filter_config_file],
            remappings=[
                ('output', 'scan'),
                ('scan', '/lidar/scan')
            ]
        )

    ld.add_action(lidar_node)
    ld.add_action(laserscan_filter_node)

    return ld
