from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions.boolean_substitution import OrSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

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
  use_fake_gripper          = OrSubstitution(use_fake_hardware, LaunchConfiguration('use_fake_gripper')).perform(context)

  include_robotiq_ft_sensor = LaunchConfiguration('include_robotiq_ft_sensor')
  include_robotiq_gripper   = LaunchConfiguration('include_robotiq_gripper')

  omron_base_ip = LaunchConfiguration('omron_base_ip').perform(context)

  xacro_path = PathJoinSubstitution([FindPackageShare('omron_imm_description'),'urdf','system.urdf.xacro']).perform(context)
  moveit_config = (MoveItConfigsBuilder('omron_imm', package_name='omron_imm_moveit_config')
                                      .robot_description(
                                        file_path=xacro_path,
                                        mappings={
                                          'use_fake_omron':               use_fake_omron,
                                          'use_fake_tm' :                 use_fake_tm,
                                          'include_robotiq_ft_sensor' :   include_robotiq_ft_sensor,
                                          'use_fake_ft_sensor' :          use_fake_ft_sensor,
                                          'include_robotiq_gripper' :     include_robotiq_gripper,
                                          'use_fake_gripper' :            use_fake_gripper,
                                          'prefix' :   f'{prefix}/',
                                          'omron_ip' :                    omron_base_ip,
                                        }
                                      )
                                      # .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
                                      # .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'pilz_industrial_motion_planner'])
                                      .to_moveit_configs()
  )


  rviz_arg = LaunchConfiguration('rviz')
  #default.rviz
  rviz_config = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'config_with_nav.rviz'])
  rviz_node = Node(
    package = 'rviz2',
    executable='rviz2',
    # namespace=ns,
    arguments=['-d',rviz_config],
    condition=IfCondition(rviz_arg),
    ros_arguments=['--log-level','warn'],
    parameters=[moveit_config.robot_description,
                moveit_config.joint_limits,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
              ],
    
    # parameters=[
    #   moveit_config.to_dict()
    # ],
    namespace=prefix,
    # remappings=[('/compute_cartesian_path', '~/compute_cartesian_path'),
    #             ('/get_planning_scene', '~/get_planning_scene'),
    #             ('/set_planner_params', '~/set_planner_params'),
    #             ('/query_planner_interface', '~/query_planner_interface'),
    #             ('/execute_trajectory', '~/execute_trajectory'),
    #             ('/move_action', '~/move_action'),
    #             ]
  )


  return [
    rviz_node,
]
