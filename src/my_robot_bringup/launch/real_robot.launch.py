#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_bus_config_path(bus_config_file: str, bus_config_dir: str) -> str:
    with open(bus_config_file, "r", encoding="utf-8") as file:
        content = file.read()

    replaced = content.replace("@BUS_CONFIG_PATH@", bus_config_dir)
    if replaced == content:
        return bus_config_file

    temp_dir = tempfile.mkdtemp(prefix="ros2_canopen_bus_")
    runtime_bus = os.path.join(temp_dir, "bus.runtime.yml")
    with open(runtime_bus, "w", encoding="utf-8") as file:
        file.write(replaced)
    return runtime_bus


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("my_robot_bringup")
    canopen_dir = get_package_share_directory("my_robot_canopen")
    description_dir = get_package_share_directory("my_robot_description")

    bus_config_dir = os.path.join(canopen_dir, "config", "robot_bus")
    bus_config_file = os.path.join(bus_config_dir, "bus.yml")
    bus_config = _resolve_bus_config_path(bus_config_file, bus_config_dir)
    master_config = os.path.join(bus_config_dir, "master.dcf")
    master_bin = os.path.join(bus_config_dir, "master.bin")
    xacro_file = os.path.join(description_dir, "urdf", "my_robot.urdf.xacro")
    controllers_file = os.path.join(bringup_dir, "config", "ros2_controllers.yaml")

    if not os.path.exists(master_bin):
        master_bin = ""

    can_interface_name = LaunchConfiguration("can_interface_name")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            '"',
            xacro_file,
            '"',
            " ",
            "bus_config:=",
            '"',
            bus_config,
            '"',
            " ",
            "master_config:=",
            '"',
            master_config,
            '"',
            " ",
            "master_bin:=",
            '"',
            master_bin,
            '"',
            " ",
            "can_interface_name:=",
            can_interface_name,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
            "--switch-timeout",
            "120",
        ],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
            "--switch-timeout",
            "120",
        ],
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=20.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_forward_position_controller_spawner = TimerAction(
        period=25.0,
        actions=[forward_position_controller_spawner],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface_name",
                default_value=TextSubstitution(text="can0"),
                description="SocketCAN interface name",
            ),
            robot_state_publisher,
            ros2_control_node,
            delayed_joint_state_broadcaster_spawner,
            delayed_forward_position_controller_spawner,
        ]
    )
