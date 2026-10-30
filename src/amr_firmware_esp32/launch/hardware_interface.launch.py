import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port of the ESP32 (tip: create a udev symlink so this never changes)",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("amr_description"),
                    "urdf",
                    "amr.urdf.xacro",
                ),
                " is_sim:=False",
                " port:=",
                LaunchConfiguration("port"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("amr_controller"),
                "config",
                "amr_controllers.yaml",
            ),
            {"robot_description": robot_description,
             "use_sim_time": False},
        ],
    )

    imu_driver_node = Node(
        package="amr_firmware_esp32",
        executable="mpu6050_driver.py"
    )

    return LaunchDescription(
        [
            port_arg,
            robot_state_publisher_node,
            controller_manager,
            imu_driver_node
        ]
    )