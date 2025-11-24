from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  args = [
    DeclareLaunchArgument(name='ns', default_value='omron', description='namespace of each node'),
    DeclareLaunchArgument(name='autostart', default_value='true', description='autostart nav nodes')

  ]

  return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  nav_params_omron = PathJoinSubstitution([FindPackageShare('omron_imm_bringup'), 'config', 'nav_params_omron.yaml'])
  pointcloud_to_laserscan_param = PathJoinSubstitution([FindPackageShare('omron_imm_bringup'), 'config', 'omron_pointcloud_to_laserscan.yaml'])

#  Omron has its own localization system
#  omron_amcl = Node(
#    package='nav2_amcl',
#    executable='amcl',
#    parameters=[nav_params_omron],
#  )

  omron_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    parameters=[nav_params_omron]
  )

  omron_behavior_server = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    parameters=[nav_params_omron],
  )

  omron_waypoint_follower = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    parameters=[nav_params_omron],
  )

  omron_planner_server = Node(
    package='nav2_planner',
    executable='planner_server',
    parameters=[nav_params_omron]
  )

  omron_controller_server = Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[nav_params_omron],
  )

  omron_smoother_server = Node(
    package='nav2_smoother',
    executable='smoother_server',
    parameters=[nav_params_omron]
  )

  #static_trasform_publisher
  map_static_trasform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['--frame-id', 'map',
               '--child-frame-id', 'omron/odom']
  )
  
  lifecycle_nodes = [
    '/omron/behavior_server',
    '/omron/bt_navigator',
    '/omron/controller_server',
    '/omron/planner_server',
    # '/omron/smoother_server',
    '/omron/waypoint_follower',
  ]

  omron_nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[{'autostart': LaunchConfiguration('autostart')}, 
                {'node_names': lifecycle_nodes}, 
                {'bond_timeout': 0.0}],
  )

  point_to_laser_cmd = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        # remappings=[('cloud_in', f'/omron/cloud_in')], # topic per laser: scan 
        parameters=[pointcloud_to_laserscan_param],
        name='pointcloud_to_laserscan'
    )

  

  omron_nav_group = GroupAction(
    actions=[PushRosNamespace(LaunchConfiguration('ns')),
#             omron_amcl,
             omron_bt_navigator,
             omron_planner_server,
             omron_controller_server,
            #  omron_smoother_server,
             omron_behavior_server,
             omron_waypoint_follower,
             omron_nav2_lifecycle_manager_node,
             map_static_trasform,
             point_to_laser_cmd
             ]
  )

  return [omron_nav_group]
