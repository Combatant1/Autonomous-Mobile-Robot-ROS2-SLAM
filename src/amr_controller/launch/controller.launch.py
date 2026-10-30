from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_cpp = Node(
        package="amr_controller",
        executable="controller_wnoise",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error},
            {"wheel_separation": wheel_separation + wheel_separation_error},
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ]
    )

    return [noisy_controller_cpp]


def generate_launch_description():

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False"
    )

    use_noisy_odom_arg = DeclareLaunchArgument(
        "use_noisy_odom",
        default_value="False",
        description="Run controller_wnoise, which publishes deliberately corrupted odometry "
                    "(simulation / EKF experiments only - never on the real robot)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    )

    # NOTE: the old default of 0.05 m was larger than the wheel radius itself (0.033 m),
    # i.e. a +150 % error. 0.005 m is a sensible "slightly wrong wheel" value.
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.239",
    )

    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_noisy_odom = LaunchConfiguration("use_noisy_odom")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amr_controller",
                "--controller-manager",
                "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
            package="controller_manager",
            executable="spawner",
            arguments=["simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),

            Node(
                package="amr_controller",
                executable="simple_controller",
                parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
            )
        ]
    )

    noisy_controller_launch = GroupAction(
        condition=IfCondition(use_noisy_odom),
        actions=[OpaqueFunction(function=noisy_controller)],
    )

    return LaunchDescription(
        [
            use_simple_controller_arg,
            use_noisy_odom_arg,
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch
        ]
    )
