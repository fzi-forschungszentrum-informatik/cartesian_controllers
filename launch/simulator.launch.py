from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    mesh_directory = PathJoinSubstitution(
        [FindPackageShare("cartesian_skill_controller"), "meshes"]
    )
    mujoco_model = PathJoinSubstitution(
        [FindPackageShare("cartesian_skill_controller"), "config", "mujoco_config.xml"]
    )

    return LaunchDescription(
        [
            Node(
                package="cartesian_skill_controller",
                executable="simulator",
                name="simulator",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"mesh_directory": mesh_directory, "mujoco_model": mujoco_model}
                ],
                # prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",  # noqa E501
            )
        ]
    )
