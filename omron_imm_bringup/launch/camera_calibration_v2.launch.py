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
                "0.0508229",
                "--y",
                "0.196913",
                "--z",
                "-0.00353158",
                "--qx",
                "0.0290704",
                "--qy",
                "0.0226338",
                "--qz",
                "0.698111",
                "--qw",
                "0.715041",
                # "--roll",
                # "0.00999811",
                # "--pitch",
                # "0.0730218",
                # "--yaw",
                # "1.54647",
            ],
        ),
    ]
    return LaunchDescription(nodes)
