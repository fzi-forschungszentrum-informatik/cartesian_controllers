from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spacenav_driver = Node(
        package="spacenav",
        executable="spacenav_node",
        parameters=[
            {"zero_when_static": True},
            {"static_count_threshold": 30},
            {"linear_scale/x": 50.0},
            {"linear_scale/y": 50.0},
            {"linear_scale/z": 50.0},
            {"angular_scale/x": 5.0},
            {"angular_scale/y": 5.0},
            {"angular_scale/z": 5.0},
        ],
        output="both",
    )

    teach_device = Node(
        package="rackki_learning",
        executable="teach_device.py",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("rackki_learning"), "config", "button_cmds.yaml"]
            ),
            {"joystick_topic": "/spacenav/joy"},
            {"twist_topic": "/spacenav/twist"},
            {"wrench_topic": "/target_wrench"},
            {"frame_id": "world"},
            {"publishing_rate": 50},
        ],
        output="both",
    )

    nodes = [
        spacenav_driver,
        teach_device,
    ]
    return LaunchDescription(nodes)
