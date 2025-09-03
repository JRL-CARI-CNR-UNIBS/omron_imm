""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: omron/base -> camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "omron/base",
                "--child-frame-id",
                "camera_link",
                "--x",
                "0.0449973",
                "--y",
                "0.204873",
                "--z",
                "-0.000623524",
                "--qx",
                "0.0141203",
                "--qy",
                "0.0133477",
                "--qz",
                "0.690804",
                "--qw",
                "0.722781",
                # "--roll",
                # "0.00197192",
                # "--pitch",
                # "0.0388134",
                # "--yaw",
                # "1.52552",
            ],
        ),
    ]
    return LaunchDescription(nodes)
