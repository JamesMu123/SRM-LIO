"""Launch the integrated SRDA + IPMM pipeline."""
import math
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory("srm_lio"))
    config = Path(arg("config")).expanduser()
    if not config.is_absolute() and not config.is_file():
        config = share / "config" / config.with_suffix(".yaml")
    config = config.resolve()
    if not config.is_file():
        raise RuntimeError(f"Configuration does not exist: {config}")
    rate, offset = float(arg("rate")), float(arg("start_offset"))
    if not math.isfinite(rate) or rate <= 0 or not math.isfinite(offset) or offset < 0:
        raise RuntimeError("rate must be positive and start_offset must be nonnegative")
    bag = Path(arg("bag")).expanduser().resolve() if arg("bag") else None
    if bag is not None and not bag.exists():
        raise RuntimeError(f"ROS 2 bag does not exist: {bag}")
    output = arg("output_dir")
    if not output:
        output = str(Path("results") / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + config.stem))
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    node = Node(
        package="srm_lio", namespace="fastlio2", executable="lio_node",
        name="lio_node", output="screen",
        parameters=[{"config_path": str(config), "experiment_dir": str(output)}],
    )
    actions = [node, RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="SRM-LIO stopped"))],
    ))]
    if arg("rviz").lower() in ("true", "1", "yes", "on"):
        actions.append(Node(
            package="rviz2", executable="rviz2", name="srm_lio_rviz", output="screen",
            arguments=["-d", str(share / "rviz" / "fastlio2.rviz")],
        ))
    if bag is not None:
        player = ExecuteProcess(cmd=[
            "ros2", "bag", "play", "--delay", "2.0", "--rate", str(rate),
            "--start-offset", str(offset), str(bag),
        ], output="screen")
        actions.extend([player, RegisterEventHandler(OnProcessExit(
            target_action=player,
            on_exit=[EmitEvent(event=Shutdown(reason="ROS 2 bag playback finished"))],
        ))])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="srm"),
        DeclareLaunchArgument("bag", default_value=""),
        DeclareLaunchArgument("output_dir", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
