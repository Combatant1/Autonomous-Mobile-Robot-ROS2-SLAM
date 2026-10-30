from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    )

    use_noisy_odom_arg = DeclareLaunchArgument(
        "use_noisy_odom",
        default_value="False",
        description="Fuse /amr_controller/odom_noisy (needs controller.launch.py use_noisy_odom:=true) "
                    "instead of the real /amr_controller/odom",
    )

    ekf_publishes_tf_arg = DeclareLaunchArgument(
        "ekf_publishes_tf",
        default_value="False",
        description="Let the EKF publish odom->base_footprint. If true you MUST also set "
                    "enable_odom_tf: false for amr_controller in amr_controllers.yaml, otherwise two "
                    "nodes publish the same transform. Leave false until you want the EKF to own odom.",
    )

    # Real wheel odometry by default; the artificially noisy one only on request.
    odom_topic = PythonExpression([
        "'amr_controller/odom_noisy' if '",
        LaunchConfiguration("use_noisy_odom"),
        "'.lower() == 'true' else 'amr_controller/odom'",
    ])

    # Needs TF base_footprint -> imu_link, which robot_state_publisher provides from the URDF
    # (the IMU driver stamps its messages with frame_id "imu_link").
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("amr_localization"), "config", "ekf.yaml"),
            {
                "odom0": odom_topic,
                "publish_tf": ParameterValue(LaunchConfiguration("ekf_publishes_tf"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_noisy_odom_arg,
        ekf_publishes_tf_arg,
        robot_localization,
    ])
