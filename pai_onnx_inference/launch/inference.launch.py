from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path",        description="Absolute path to .onnx (required)"),
        DeclareLaunchArgument("contract_path",     description="Absolute path to contract YAML (required)"),
        DeclareLaunchArgument("ep_requested",      default_value="auto"),
        DeclareLaunchArgument("ep_policy",         default_value="fallback"),
        DeclareLaunchArgument("observation_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("drop_warn_threshold",   default_value="5"),
        DeclareLaunchArgument("onnx_error_policy",     default_value="log_and_drop"),
        DeclareLaunchArgument("sim_time_watchdog_s",   default_value="2.0"),
        DeclareLaunchArgument("use_sim_time",          default_value="false"),

        Node(
            package="pai_onnx_inference",
            executable="pai_onnx_inference_node",
            name="pai_onnx_inference",
            output="screen",
            parameters=[{
                "model_path":        LaunchConfiguration("model_path"),
                "contract_path":     LaunchConfiguration("contract_path"),
                "ep_requested":      LaunchConfiguration("ep_requested"),
                "ep_policy":         LaunchConfiguration("ep_policy"),
                "observation_timeout_s": LaunchConfiguration("observation_timeout_s"),
                "drop_warn_threshold":   LaunchConfiguration("drop_warn_threshold"),
                "onnx_error_policy":     LaunchConfiguration("onnx_error_policy"),
                "sim_time_watchdog_s":   LaunchConfiguration("sim_time_watchdog_s"),
                "use_sim_time":          LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
