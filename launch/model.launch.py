from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg_model_path = DeclareLaunchArgument(
        "model_path",
        default_value="/home/scherzin/src/robot_folders/checkout/rack-ki/colcon_ws/src/rackki_learning/test/models/model_1",  # noqa: E501
        description="Absolute path to the trained model.",
    )
    return LaunchDescription(
        [
            arg_model_path,
            Node(
                package="rackki_learning",
                executable="model.py",
                name="model",
                output="screen",
                emulate_tty=True,
                parameters=[{"model_path": LaunchConfiguration("model_path")}],
            ),
        ]
    )
