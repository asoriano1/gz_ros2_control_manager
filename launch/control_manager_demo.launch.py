#!/usr/bin/env python3
"""Demo launcher for gz_ros2_control_manager.

Default behaviour (no arguments):
  Spawns the bundled diffbot (differential-drive robot) in Gazebo Harmonic
  with gz_ros2_control enabled.  The gz_ros2_control_manager GUI panel
  discovers /diffbot/controller_manager and shows joint_state_broadcaster
  and diff_drive_controller as "Configured (not loaded)".

  Use the GUI to:
    1. Load inactive  joint_state_broadcaster
    2. Activate       joint_state_broadcaster
    3. Load inactive  diff_drive_controller
    4. Activate       diff_drive_controller

  Then move the robot (Jazzy: diff_drive_controller uses TwistStamped):
    ros2 topic pub /diffbot/diff_drive_controller/cmd_vel \\
        geometry_msgs/msg/TwistStamped \\
        "{header: {stamp: {sec: 0}, frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}" -r 10

Optional: supply your own robot via robot_xacro_path:
    ros2 launch gz_ros2_control_manager control_manager_demo.launch.py \\
        robot_xacro_path:=/path/to/robot.urdf.xacro namespace:=my_robot

Robot selection (priority order):
  1. robot_xacro_path:=<path>          — explicit file, no extra deps
  2. bundled diffbot                   — default, always available
  3. robotnik_description (rbvogui)    — legacy fallback if installed
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


PACKAGE_NAME = "gz_ros2_control_manager"


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _resolve_robot_xacro(context) -> str | None:
    """Return an absolute path to a robot xacro/urdf, or None on failure.

    Priority:
      1. Explicit robot_xacro_path argument.
      2. Bundled diffbot (always available inside this package).
      3. robotnik_description fallback (optional, backward-compat).
    """
    explicit = LaunchConfiguration("robot_xacro_path").perform(context).strip()
    if explicit:
        return os.path.expanduser(explicit)

    # 2. Bundled diffbot — primary default.
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    bundled = os.path.join(pkg_share, "urdf", "diffbot.urdf.xacro")
    if os.path.isfile(bundled):
        return bundled

    # 3. robotnik_description fallback (optional).
    robot       = LaunchConfiguration("robot").perform(context).strip()
    robot_model = LaunchConfiguration("robot_model").perform(context).strip() or robot
    try:
        rd_share  = get_package_share_directory("robotnik_description")
        candidate = os.path.join(rd_share, "robots", robot, f"{robot_model}.urdf.xacro")
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass

    return None


def _launch_setup(context, *args, **kwargs):
    pkg_share      = get_package_share_directory(PACKAGE_NAME)
    pkg_lib        = os.path.join(get_package_prefix(PACKAGE_NAME), "lib")
    gui_config_path = os.path.join(pkg_share, "config", "control_manager.config")
    default_world_path = os.path.join(pkg_share, "worlds", "control_manager_demo.sdf")

    world_arg  = LaunchConfiguration("world").perform(context).strip()
    world_path = os.path.expanduser(world_arg) if world_arg else default_world_path

    gui_enabled  = _as_bool(LaunchConfiguration("gui").perform(context))
    start_paused = _as_bool(LaunchConfiguration("paused").perform(context))
    bridge_clock = _as_bool(LaunchConfiguration("bridge_clock").perform(context))

    namespace    = LaunchConfiguration("namespace").perform(context)
    frame_prefix = LaunchConfiguration("frame_prefix").perform(context)
    spawn_x      = LaunchConfiguration("x").perform(context)
    spawn_y      = LaunchConfiguration("y").perform(context)
    spawn_z      = LaunchConfiguration("z").perform(context)

    actions = [
        LogInfo(msg=f"Control Manager lib dir: {pkg_lib}"),
        LogInfo(msg=f"GUI config path:          {gui_config_path}"),
        LogInfo(msg=f"Demo world:               {world_path}"),
        SetEnvironmentVariable(
            name="GZ_GUI_PLUGIN_PATH",
            value=[
                pkg_lib,
                ":",
                EnvironmentVariable("GZ_GUI_PLUGIN_PATH", default_value=""),
            ],
        ),
    ]

    robot_xacro_path = _resolve_robot_xacro(context)
    if not robot_xacro_path:
        actions.append(
            LogInfo(
                msg=(
                    "No robot_xacro_path was provided, the bundled diffbot was not "
                    "found, and robotnik_description is not available.\n"
                    "Pass robot_xacro_path:=/abs/path/to/robot.urdf.xacro.\n"
                    "Gazebo will not be launched."
                )
            )
        )
        return actions

    actions.append(LogInfo(msg=f"Robot xacro:    {robot_xacro_path}"))
    actions.append(LogInfo(msg=f"Robot namespace: /{namespace}"))

    # Make gz_ros2_control's Gazebo system plugin discoverable.
    try:
        gz_ros2_control_lib = os.path.join(
            get_package_prefix("gz_ros2_control"), "lib"
        )
        actions.extend(
            [
                LogInfo(msg=f"gz_ros2_control lib dir: {gz_ros2_control_lib}"),
                SetEnvironmentVariable(
                    name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                    value=[
                        gz_ros2_control_lib,
                        ":",
                        EnvironmentVariable(
                            "GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""
                        ),
                    ],
                ),
            ]
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "gz_ros2_control was not found. The spawned robot will not "
                    "start a controller_manager node.\n"
                    "Install ros-jazzy-gz-ros2-control or build it from source."
                )
            )
        )

    # Optional: bridge /clock so controller_manager can use sim time.
    if bridge_clock:
        try:
            ros_gz_bridge_prefix = get_package_prefix("ros_gz_bridge")
            clock_bridge_exec = os.path.join(
                ros_gz_bridge_prefix, "lib", "ros_gz_bridge", "parameter_bridge"
            )
            if os.path.exists(clock_bridge_exec):
                actions.append(
                    ExecuteProcess(
                        cmd=[
                            clock_bridge_exec,
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        ],
                        output="screen",
                    )
                )
            else:
                actions.append(
                    LogInfo(msg="ros_gz_bridge parameter_bridge not found; no /clock bridge.")
                )
        except PackageNotFoundError:
            actions.append(
                LogInfo(msg="ros_gz_bridge not found; no /clock bridge.")
            )

    # Build robot_description from the xacro.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_xacro_path,
            " namespace:=",
            namespace,
            " prefix:=",
            frame_prefix,
            " gazebo_ignition:=true",
        ]
    )
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description_param,
                    "publish_frequency": 50.0,
                    "use_sim_time": True,
                    "frame_prefix": frame_prefix,
                }
            ],
        )
    )

    # Launch Gazebo with the demo world.
    gz_cmd = ["gz", "sim", "-v", "3"]
    if not start_paused:
        gz_cmd.append("-r")
    if gui_enabled:
        gz_cmd.extend(["--gui-config", gui_config_path])
    else:
        gz_cmd.append("-s")
    gz_cmd.append(world_path)
    gz_sim_proc = ExecuteProcess(cmd=gz_cmd, output="screen")
    actions.append(gz_sim_proc)

    # When Gazebo exits (e.g. window X button), shut down the whole session.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_sim_proc,
                on_exit=[
                    LogInfo(msg="Gazebo Sim exited — shutting down launch."),
                    EmitEvent(event=ShutdownEvent(reason="Gazebo Sim closed")),
                ],
            )
        )
    )

    # Spawn robot from the robot_description topic.
    try:
        get_package_prefix("ros_gz_sim")
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", namespace,
                    "-topic", f"/{namespace}/robot_description",
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                    "-allow_renaming", "true",
                ],
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "ros_gz_sim not found, cannot spawn the robot. "
                    "Install ros-jazzy-ros-gz-sim."
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_xacro_path",
                default_value="",
                description=(
                    "Absolute path to a xacro/urdf to spawn. "
                    "If empty, uses the bundled diffbot demo robot."
                ),
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="rbvogui",
                description=(
                    "robotnik_description robot family (legacy fallback only)."
                ),
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value="",
                description=(
                    "robotnik_description robot variant (legacy fallback only)."
                ),
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="diffbot",
                description=(
                    "ROS namespace for robot_state_publisher and "
                    "controller_manager. Must match the <ros><namespace> tag "
                    "in the robot xacro."
                ),
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="TF frame prefix.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="",
                description="Override world SDF. Uses the bundled demo world if empty.",
            ),
            DeclareLaunchArgument("x", default_value="0.0",
                                  description="Spawn X coordinate (m)."),
            DeclareLaunchArgument("y", default_value="0.0",
                                  description="Spawn Y coordinate (m)."),
            DeclareLaunchArgument(
                "z", default_value="0.11",
                description=(
                    "Spawn Z coordinate (m). Default 0.11 m for diffbot "
                    "(wheel_radius=0.10 + small clearance)."
                ),
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the Gazebo GUI with the control manager panel.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start simulation paused.",
            ),
            DeclareLaunchArgument(
                "bridge_clock",
                default_value="true",
                description="Bridge /clock from Gazebo so use_sim_time works.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
