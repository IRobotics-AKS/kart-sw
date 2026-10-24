"""Discoverable launch contract; refuses execution until runtime exists."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _require_implementation(context):
    # Autonomy will support both clocks using the same implementation.
    raise RuntimeError(
        "autonomy_bringup is scaffold-only: runtime nodes and validated "
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
            "enable_perception", default_value="false",
            description="Enable an implemented perception pipeline.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "trajectory_source", default_value="planner",
            description="Planner or explicitly selected test source.",
            choices=["planner", "test"],
        ),
        OpaqueFunction(function=_require_implementation),
    ])
