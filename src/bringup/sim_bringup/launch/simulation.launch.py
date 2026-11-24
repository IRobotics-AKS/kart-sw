"""Launch only the MuJoCo bridge; no physical command path."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    if value("use_sim_time").lower() != "true":
        raise RuntimeError("Simulation bringup requires use_sim_time=true.")
    if not value("model"):
        raise RuntimeError("Select model:=demo or provide an MJCF model path.")
    config = value("config_file")
    if not config or not Path(config).is_file():
        raise RuntimeError("config_file must name a ROS parameter YAML file; see config/bridge_demo.yaml.")
    for unsupported in ("scenario", "sensor_config"):
        if value(unsupported):
            raise RuntimeError(f"{unsupported} is not implemented; configure the bridge using config_file.")
    return [Node(
        package="sim_ros", executable="sim_bridge", name="sim_bridge",
        namespace=value("namespace"), output="screen",
        parameters=[config, {
            "use_sim_time": True, "model": value("model"),
            "seed": int(value("seed")), "timestep": float(value("timestep")),
            "real_time_factor": float(value("real_time_factor")),
            "rendering": value("rendering").lower() == "true",
        }],
        on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo bridge exited"))],
    )]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("namespace", default_value="kart"),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("model", default_value="", description="MJCF path or explicit 'demo' test fixture"),
        DeclareLaunchArgument("config_file", default_value="", description="Bridge ROS parameter YAML"),
        DeclareLaunchArgument("seed", default_value="0", description="Reproducible sensor-noise seed"),
        DeclareLaunchArgument("timestep", default_value="0.0", description="Seconds; zero preserves the MJCF timestep"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0", description="Requested pacing, not a deadline guarantee"),
        DeclareLaunchArgument("rendering", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scenario", default_value="", description="Reserved; nonempty values rejected"),
        DeclareLaunchArgument("sensor_config", default_value="", description="Reserved; use config_file"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch)])
