import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_noisy_odom_arg = DeclareLaunchArgument(
        "use_noisy_odom",
        default_value="False",
        description="Also run controller_wnoise (artificially noisy odometry, for EKF experiments)",
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_noisy_odom": LaunchConfiguration("use_noisy_odom"),
            "use_sim_time": "True",
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "joystick.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    return LaunchDescription([
        use_noisy_odom_arg,
        gazebo,
        controller,
        joystick,
    ])
