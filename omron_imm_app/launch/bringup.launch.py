from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_spawn_controllers_launch

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument('use_fake_hardware', default_value='true'),
    DeclareLaunchArgument('rviz',              default_value='true'),
    DeclareLaunchArgument('ns',                default_value='omron'),
    DeclareLaunchArgument('use_gripper',       default_value='true'),
    DeclareLaunchArgument('use_moveit',        default_value='true'),
  ]

  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

  fake_hardware = LaunchConfiguration('use_fake_hardware')
  use_gripper =   LaunchConfiguration('use_gripper')
  xacro_path = PathJoinSubstitution([FindPackageShare("omron_imm_description"),"urdf","system.urdf.xacro"])
  moveit_config = (MoveItConfigsBuilder("omron_imm", package_name="omron_imm_moveit_config")
                                      .robot_description(
                                        file_path=xacro_path.perform(context),
                                        mappings={"fake":fake_hardware, "use_gripper":use_gripper},
                                      )
                                      .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
                                      .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
                                      .to_moveit_configs()
  )

  return [
          *launch_moveit(context, moveit_config),
          *launch_omron(context, moveit_config.robot_description),
          *launch_gripper(context)
          ]


def launch_moveit(context, moveit_config):

  use_moveit = LaunchConfiguration('use_moveit')

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    # namespace=ns,
    condition=IfCondition(use_moveit),
    output="screen",
    parameters=[moveit_config.to_dict()],
    arguments=["--ros-args", "--log-level", "info"],
    remappings=[("/joint_states", "/omron/joint_states")]
  )

  rviz_arg = LaunchConfiguration('rviz')

  rviz_config = PathJoinSubstitution([FindPackageShare("omron_imm_app"),"rviz","default.rviz"])
  rviz_node = Node(
    package = "rviz2",
    executable="rviz2",
    # namespace=ns,
    arguments=["-d",rviz_config],
    condition=IfCondition(rviz_arg),
    parameters=[
      moveit_config.to_dict()
    ]
  )

  return [move_group_node, rviz_node]


def launch_omron(context, robot_description: str):

  ns = LaunchConfiguration('ns')

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=ns,
    output="both",
    parameters=[robot_description],
  )

  # ros2_controllers_path = PathJoinSubstitution([
  #   FindPackageShare("omron_imm_moveit_config"),
  #   "config",
  #   "ros2_controllers.yaml"
  # ])

  ld60_params = PathJoinSubstitution([
    FindPackageShare("omron_imm_description"),
    "config",
    "omron_ld60.yaml"
  ])

  ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    # name="controller_manager",
    namespace=ns,
    parameters=[ld60_params, robot_description],
    output="screen",
  )

  controller_manager_name = f"/{ns.perform(context)}/controller_manager"

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    namespace=ns,
    arguments=[
      "joint_state_broadcaster",
      "-c", controller_manager_name
    ],
  )

  tm12_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    namespace=ns,
    arguments=["joint_trajectory_controller",
               "-c", controller_manager_name
               ],
    output="screen"
  )

  fake_hardware = LaunchConfiguration('use_fake_hardware')
  support_nodes = Node(
    package="omron_hardware_interface",
    executable="omron_support_nodes",
    namespace=ns,
    parameters=[ld60_params],
    output="screen",
    condition=LaunchConfigurationEquals(fake_hardware, "false")
  )

  omron_state_bcast_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["omron_state_broadcaster",
               "-c", controller_manager_name],
    condition=LaunchConfigurationEquals(fake_hardware, "false")
  )

  return [robot_state_publisher_node,
    ros2_control_node,
    joint_state_broadcaster_spawner,
    tm12_controller_spawner,
    support_nodes,
    omron_state_bcast_spawner
    ]


def launch_gripper(context):

  use_gripper = LaunchConfiguration('use_gripper')
  ns = LaunchConfiguration('ns')

  controller_manager_name = f"/{ns.perform(context)}/controller_manager"

    # gripper_launch = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(
  #     PathJoinSubstitution([
  #       FindPackageShare('omron_imm_app'), 
  #       'launch', 
  #       'gripper.launch.py'
  #     ])
  #   ),
  #   # launch_arguments={'robot_description': robot_description}.items(),
  # )

  robotiq_gripper_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["robotiq_gripper_controller", 
                 "--controller-manager", controller_manager_name],
      condition=IfCondition(use_gripper)
  )

  robotiq_activation_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["robotiq_activation_controller", 
                 "--controller-manager", controller_manager_name],
      condition=IfCondition(use_gripper)
  )

  return [
    robotiq_gripper_controller_spawner,
    robotiq_activation_controller_spawner
  ]
