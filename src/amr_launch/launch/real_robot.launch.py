import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port the ESP32 is connected to",
    )

    # Starts robot_state_publisher, ros2_control_node (with the ESP32 hardware
    # interface) and the MPU6050 driver. The IMU driver lives ONLY here - do not
    # start it a second time from this file.
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_firmware_esp32"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={"port": LaunchConfiguration("port")}.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_noisy_odom": "False",   # never feed artificial noise on the real robot
            "use_sim_time": "False",
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_controller"),
            "launch",
            "joystick.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("amr_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "use_noisy_odom": "False",
        }.items()
    )

    return LaunchDescription([
        port_arg,
        hardware_interface,
        controller,
        joystick,
        localization,
    ])
