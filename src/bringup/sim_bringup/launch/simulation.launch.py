"""Discoverable launch contract; refuses execution until runtime exists."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _require_implementation(context):
    if LaunchConfiguration("use_sim_time").perform(context).lower() != "true":
        raise RuntimeError("Simulation bringup requires use_sim_time=true.")
    raise RuntimeError(
        "sim_bringup is scaffold-only: runtime nodes and validated "
        "configuration are not implemented. No processes have been started."
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="kart",
            description="Vehicle namespace; frame IDs are configured separately.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the simulator clock.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "config_file", default_value="",
            description="Path to reviewed runtime parameters; not available in the scaffold.",
        ),
        DeclareLaunchArgument(
            "model", default_value="",
            description="Path to measured MJCF.",
        ),
        DeclareLaunchArgument(
            "scenario", default_value="",
            description="Path to a simulation scenario.",
        ),
        DeclareLaunchArgument(
            "sensor_config", default_value="",
            description="Path to simulated sensor configuration.",
        ),
        DeclareLaunchArgument(
            "seed", default_value="",
            description="Explicit noise seed.",
        ),
        DeclareLaunchArgument(
            "timestep", default_value="",
            description="Physics step in seconds; no value selected.",
        ),
        DeclareLaunchArgument(
            "rendering", default_value="false",
            description="Enable an implemented rendering backend.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "real_time_factor", default_value="",
            description="Requested wall-clock pacing; no value selected.",
        ),
        OpaqueFunction(function=_require_implementation),
    ])
