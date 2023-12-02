from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg_model_dir = DeclareLaunchArgument(
        "model_dir",
        default_value="/home/scherzin/src/robot_folders/checkout/rack-ki/colcon_ws/src/rackki_learning/test/models/model_1",  # noqa: E501
        description="Absolute path to the trained model directory.",
    )
    return LaunchDescription(
        [
            arg_model_dir,
            Node(
                package="rackki_learning",
                executable="serve_model.py",
                name="model",
                output="screen",
                emulate_tty=True,
                parameters=[{"model_dir": LaunchConfiguration("model_dir")}],
            ),
        ]
    )
