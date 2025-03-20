from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions.boolean_substitution import OrSubstitution
import yaml
import os

package_name = 'omron_imm_bringup'

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument(name='prefix',                    default_value='omron',   description='namespace of each node + tf_prefix'),
    DeclareLaunchArgument(name='use_fake_hardware',         default_value='false',   description='use mock for all components'),
    DeclareLaunchArgument(name='use_fake_omron',            default_value='false',   description='use mock for omron control'),
    DeclareLaunchArgument(name='use_fake_tm',               default_value='false',   description='use mock for tm control'),
    DeclareLaunchArgument(name='include_robotiq_ft_sensor', default_value='true',   description='add robotiq ft sensor to urdf'),
    DeclareLaunchArgument(name='use_fake_ft_sensor',        default_value='false',   description='use mock for ft sensor control'),
    DeclareLaunchArgument(name='include_robotiq_gripper',   default_value='true',   description='add robotiq gripper to urdf'),
    DeclareLaunchArgument(name='use_fake_gripper',          default_value='true',   description='use mock for gripper'),
    DeclareLaunchArgument(name='omron_base_ip',             default_value='1.2.3.4', description='omron base ip address'),
    DeclareLaunchArgument(name='rviz',                      default_value='true',    description='launch rviz'),
  ]

  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
  prefix = LaunchConfiguration('prefix').perform(context)

  use_fake_hardware         = LaunchConfiguration('use_fake_hardware')
  use_fake_omron            = OrSubstitution(use_fake_hardware, LaunchConfiguration('use_fake_omron')).perform(context)
  use_fake_tm               = OrSubstitution(use_fake_hardware, LaunchConfiguration('use_fake_tm')).perform(context)
  use_fake_ft_sensor        = OrSubstitution(use_fake_hardware, LaunchConfiguration('use_fake_ft_sensor')).perform(context)
  use_fake_gripper          = OrSubstitution(use_fake_hardware, LaunchConfiguration('use_fake_ft_sensor')).perform(context)

  include_robotiq_ft_sensor = LaunchConfiguration('include_robotiq_ft_sensor')
  include_robotiq_gripper   = LaunchConfiguration('include_robotiq_gripper')

  omron_base_ip = LaunchConfiguration('omron_base_ip').perform(context)

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name='xacro')]),
          ' ',
          PathJoinSubstitution([FindPackageShare('omron_imm_description'),'urdf','system.urdf.xacro']),
          ' ',
          'use_fake_omron:=', use_fake_omron,
          ' ',
          'use_fake_tm:=', use_fake_tm,
          ' ',
          'include_robotiq_ft_sensor:=', include_robotiq_ft_sensor,
          ' ',
          'use_fake_ft_sensor:=', use_fake_ft_sensor,
          ' ',
          'include_robotiq_gripper:=', include_robotiq_gripper,
          ' ',
          'use_fake_gripper:=', use_fake_gripper,
          ' ',
          'prefix:=', f'{prefix}/',
          ' ',
          'omron_ip:=', omron_base_ip,
      ]
  )

  rviz_config = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'default.rviz'])
  rviz_node = Node(
    package = 'rviz2',
    executable='rviz2',
    # namespace=ns,
    arguments=['-d',rviz_config],
    ros_arguments=['--log-level','warn'],
  )
  return [rviz_node]


