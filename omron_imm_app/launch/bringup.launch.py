from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_spawn_controllers_launch

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument('fake_hardware', default_value='true'),
    DeclareLaunchArgument('rviz', default_value='true'),
    DeclareLaunchArgument('ns', default_value='omron'),
  ]

  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

  fake_hardware = LaunchConfiguration('fake_hardware')
  rviz_arg = LaunchConfiguration('rviz')
  ns = LaunchConfiguration('ns')

  print("Fake: " + fake_hardware.perform(context))
  print("Rviz: " + rviz_arg.perform(context))

  xacro_path = PathJoinSubstitution([FindPackageShare("omron_imm_description"),"urdf","system.urdf.xacro"])
  moveit_config = (MoveItConfigsBuilder("omron_imm", package_name="omron_imm_moveit_config")
                                      .robot_description(
                                        file_path=xacro_path.perform(context),
                                        mappings={"fake":fake_hardware},
                                      )
                                      .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
                                      .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
                                      .to_moveit_configs()
  )

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    # namespace=ns,
    output="screen",
    parameters=[moveit_config.to_dict()],
    arguments=["--ros-args", "--log-level", "info"],
    remappings=[("/joint_states", "/omron/joint_states")]
  )

  rviz_config = PathJoinSubstitution([FindPackageShare("omron_imm_app"),"rviz","default.rviz"])
  rviz_node = Node(
    package = "rviz2",
    executable="rviz2",
    name="rviz2",
    # namespace=ns,
    arguments=["-d",rviz_config],
    condition=IfCondition(rviz_arg),
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.planning_pipelines,
      moveit_config.robot_description_kinematics,
      moveit_config.joint_limits,
    ]
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=ns,
    output="both",
    parameters=[moveit_config.robot_description],
  )

  ros2_controllers_path = PathJoinSubstitution([
    FindPackageShare("omron_imm_moveit_config"),
    "config",
    "ros2_controllers.yaml"
  ])

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          xacro_path,
          " fake:=",fake_hardware,
      ]
  )
  robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


  # joint_state_publisher_node = Node(
  #   package="joint_state_publisher",
  #   executable="joint_state_publisher",
  #   name="joint_state_publisher",
  # )

  ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    # name="controller_manager",
    namespace=ns,
    parameters=[ros2_controllers_path, robot_description],
    output="screen",
  )

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    namespace=ns,
    arguments=[
      "joint_state_broadcaster",
      # "-c", f"{ns.perform(context)}/controller_manager"
    ],
  )

  tm12_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    namespace=ns,
    name="tm12_controller_spawner",
    arguments=["imm_controller", "-p", ros2_controllers_path],
              #  "-c", f"{ns.perform(context)}/controller_manager"],
    output="screen"
  )

  return [
    move_group_node,
    rviz_node,
    robot_state_publisher_node,
    # joint_state_publisher_node,
    ros2_control_node,
    joint_state_broadcaster_spawner,
    tm12_controller_spawner
  ]
