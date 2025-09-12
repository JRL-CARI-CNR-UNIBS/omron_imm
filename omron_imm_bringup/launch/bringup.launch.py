from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions.boolean_substitution import OrSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from nav2_common.launch import RewrittenYaml
import xacro

package_name = 'omron_imm_bringup'

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument(name='prefix',                    default_value='omron',   description='namespace of each node + tf_prefix'),
    DeclareLaunchArgument(name='use_fake_hardware',         default_value='false',   description='use mock for all components'),
    DeclareLaunchArgument(name='use_fake_omron',            default_value='false',   description='use mock for omron control'),
    DeclareLaunchArgument(name='use_fake_tm',               default_value='false',   description='use mock for tm control'),
    DeclareLaunchArgument(name='include_robotiq_ft_sensor', default_value='false',   description='add robotiq ft sensor to urdf'),
    DeclareLaunchArgument(name='use_fake_ft_sensor',        default_value='false',   description='use mock for ft sensor control'),
    DeclareLaunchArgument(name='include_robotiq_gripper',   default_value='false',   description='add robotiq gripper to urdf'),
    DeclareLaunchArgument(name='use_fake_gripper',          default_value='false',   description='use mock for gripper'),
    DeclareLaunchArgument(name='omron_base_ip',             default_value='1.2.3.4', description='omron base ip address'),
    DeclareLaunchArgument(name='use_moveit',                default_value='true',    description='launch move_group'),
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

  include_robotiq_ft_sensor = LaunchConfiguration('include_robotiq_ft_sensor').perform(context)
  include_robotiq_gripper   = LaunchConfiguration('include_robotiq_gripper').perform(context)

  omron_base_ip = LaunchConfiguration('omron_base_ip').perform(context)

  robot_description_mappings = {
                                  'use_fake_omron':               use_fake_omron,
                                  'use_fake_tm' :                 use_fake_tm,
                                  'include_robotiq_ft_sensor' :   include_robotiq_ft_sensor,
                                  'use_fake_ft_sensor' :          use_fake_ft_sensor,
                                  'include_robotiq_gripper' :     include_robotiq_gripper,
                                  'use_fake_gripper' :            use_fake_gripper,
                                  'prefix' :                      f'{prefix}/',
                                  'omron_ip' :                    omron_base_ip,
                                }

  xacro_path = PathJoinSubstitution([FindPackageShare('omron_imm_description'),'urdf','system.urdf.xacro']).perform(context)
  urdf = xacro.process(xacro_path,
         mappings=robot_description_mappings)

  moveit_config = (MoveItConfigsBuilder('omron_imm', package_name='omron_imm_moveit_config')
                                      .robot_description(
                                        file_path=xacro_path,
                                        mappings=robot_description_mappings
                                      )
                                      .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
                                      # .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'pilz_industrial_motion_planner'])
                                      .to_moveit_configs()
  )

  use_moveit = LaunchConfiguration('use_moveit')
  planning_pipelines_config = PathJoinSubstitution(
    [
        FindPackageShare('omron_imm_moveit_config'),
        "config",
        "planning_pipelines.yaml",
    ]
  )

  move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    condition=IfCondition(use_moveit),
    output='screen',
    parameters=[moveit_config.to_dict(),
                planning_pipelines_config],
    arguments=['--ros-args', '--log-level', 'info'],
    # remappings=[('joint_states',f'{prefix}/joint_states')],
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[moveit_config.robot_description],
  )

  rviz_arg = LaunchConfiguration('rviz')

  rviz_config = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'default.rviz'])
  rviz_node = Node(
    package = 'rviz2',
    executable='rviz2',
    # namespace=ns,
    arguments=['-d',rviz_config],
    condition=IfCondition(rviz_arg),
    ros_arguments=['--log-level','warn'],
    parameters=[
      moveit_config.to_dict()
    ]
  )

  original_controller_config = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'ros2_controllers.yaml'])
  config_rewrite = RewrittenYaml(
    source_file=original_controller_config,
    param_rewrites={'robot_description' : urdf}
  )

  ros2_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[config_rewrite],
    output='screen',
    # arguments=['--ros-args', '--log-level', 'debug'],
    remappings=[('~/robot_description', 'robot_description')],
  )

  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
  )

  tm12_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_trajectory_controller',
              '--ros-args', '--log-level', 'debug'],
    output='screen'
  )

  support_nodes = Node(
    package='omron_hardware_interface',
    executable='omron_support_nodes',
    parameters=[config_rewrite],
    output='screen',
    condition=UnlessCondition(use_fake_omron),
    arguments=[omron_base_ip]
  )

  omron_state_bcast_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['omron_state_broadcaster'],
    condition=UnlessCondition(use_fake_omron)
  )

  robotiq_gripper_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["robotiq_gripper_controller"],
  )

  robotiq_activation_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["robotiq_activation_controller"],
      condition=UnlessCondition(use_fake_gripper)
  )

  # omron_fake_pose_spawner = Node(
  #   package='controller_manager',
  #   executable='spawner',
  #   arguments=['omron_fake_pose_controller'],
  #   condition=IfCondition(use_fake_omron),
  # )  

  cnr_admittance_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['cnr_admittance_controller',
    '-p', config_rewrite,
    '-t', 'cnr_admittance_controller/AdmittanceController',
    '--inactive'],
  )

  omron_forward_controller = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['omron_forward_controller'],
  )

  ft_sensor_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['ft_sensor_broadcaster'],
    condition=UnlessCondition(use_fake_ft_sensor),
  )

  return [
    rviz_node,
    GroupAction(
      actions=[
        PushRosNamespace(prefix),
        move_group_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        tm12_controller_spawner,
        support_nodes,
        omron_state_bcast_spawner,
        omron_forward_controller,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        cnr_admittance_controller_spawner,
        ft_sensor_broadcaster_spawner
    ])
]
