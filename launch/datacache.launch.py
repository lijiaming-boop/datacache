from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path',
                              default_value=PathJoinSubstitution([
                                  FindPackageShare('datacache'), 'config.txt']),
                              description='Path to config file'),
        DeclareLaunchArgument('pcd_path',
                              default_value=PathJoinSubstitution([
                                  FindPackageShare('datacache'), 'pcd', 'sample.pcd']),
                              description='Path to PCD file for lidar simulation'),
        DeclareLaunchArgument('camera_fps', default_value='30',
                              description='Camera capture frame rate'),
        DeclareLaunchArgument('camera_device', default_value='0',
                              description='Camera device index'),
        DeclareLaunchArgument('lidar_hz', default_value='10',
                              description='Lidar point cloud publish rate'),
        DeclareLaunchArgument('event_interval', default_value='0',
                              description='Auto-trigger interval in seconds (0=manual only)'),

        Node(
            package='datacache',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'fps': LaunchConfiguration('camera_fps'),
                'device': LaunchConfiguration('camera_device'),
            }],
            output='screen',
        ),

        Node(
            package='datacache',
            executable='lidar_sim_node',
            name='lidar_sim_node',
            parameters=[{
                'pcd_path': LaunchConfiguration('pcd_path'),
                'hz': LaunchConfiguration('lidar_hz'),
            }],
            output='screen',
        ),

        Node(
            package='datacache',
            executable='datacache_node',
            name='datacache_node',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
            }],
            output='screen',
        ),

        Node(
            package='datacache',
            executable='event_trigger_node',
            name='event_trigger_node',
            parameters=[{
                'interval': LaunchConfiguration('event_interval'),
            }],
            output='screen',
        ),
    ])
