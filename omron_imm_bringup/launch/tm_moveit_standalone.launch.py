from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  config_path = DeclareLaunchArgument(
    name='config_path',
    default_value=PathJoinSubstitution([FindPackageShare('omron_imm_bringup'), 'config', 'tm_moveit_standalone.yaml']),
    description='Full path to the config file to load'
  )
  namespace = DeclareLaunchArgument(
    name='namespace',
    default_value='omron',
    description='Namespace of each node'
  )

  tm_moveit_standalone = Node(
    package='stiima_tm12_hw',
    executable='tm_moveit_standalone',
    name='tm_moveit_standalone_node',
    output='screen',
    parameters=[LaunchConfiguration('config_path')],
    namespace=LaunchConfiguration('namespace')
  )
  return LaunchDescription([config_path, namespace, tm_moveit_standalone])