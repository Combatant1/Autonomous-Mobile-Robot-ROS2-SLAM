from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock (needed so TwistStamped stamps match Gazebo time)",
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("amr_controller"), "config", "joy_config.yaml"),
            {"use_sim_time": use_sim_time},
        ]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(get_package_share_directory("amr_controller"), "config", "joy_teleop.yaml"),
            {"use_sim_time": use_sim_time},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop
    ])
