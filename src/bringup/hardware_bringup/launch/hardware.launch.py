"""Discoverable launch contract; refuses execution until runtime exists."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _require_implementation(context):
    if LaunchConfiguration("use_sim_time").perform(context).lower() != "false":
        raise RuntimeError("Physical hardware bringup rejects use_sim_time=true.")
    raise RuntimeError(
        "hardware_bringup is scaffold-only: runtime nodes and validated "
        "configuration are not implemented. No processes have been started."
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="kart",
            description="Vehicle namespace; frame IDs are configured separately.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use the simulator clock.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "config_file", default_value="",
            description="Path to reviewed runtime parameters; not available in the scaffold.",
        ),
        DeclareLaunchArgument(
            "device_config", default_value="",
            description="Stable sensor and MCU device mapping.",
        ),
        DeclareLaunchArgument(
            "calibration", default_value="",
            description="Measured sensor calibration.",
        ),
        DeclareLaunchArgument(
            "gnss_corrections", default_value="",
            description="External correction configuration; keep credentials out of Git.",
        ),
        DeclareLaunchArgument(
            "mcu_transport", default_value="",
            description="Reviewed MCU transport configuration.",
        ),
        OpaqueFunction(function=_require_implementation),
    ])
