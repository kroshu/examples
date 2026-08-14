# Copyright 2026 KUKA Hungaria Kft.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

# Multi-robot configuration constants
NUM_ROBOTS = 2
ROBOT_INDICES = range(1, NUM_ROBOTS + 1)
ROBOT_PREFIXES = [f"robot{i}" for i in ROBOT_INDICES]
SINGLE_ARM_GROUP = "manipulator"


def _remap_config_for_robots(base_cfg: dict, group_name: str = SINGLE_ARM_GROUP) -> dict:
    """Remap single-arm config to multi-robot groups with prefixed names."""
    base_group_cfg = base_cfg.get(group_name, {})
    if not base_group_cfg:
        return {}
    return {f"{prefix}_{group_name}": dict(base_group_cfg) for prefix in ROBOT_PREFIXES}


def _prefix_joint_names(joint_dict: dict) -> tuple[dict, list]:
    """Prefix joint names for all robots and return dict and flat list."""
    prefixed_joints = {}
    prefixed_names = []
    for joint_name, limits in joint_dict.items():
        for prefix in ROBOT_PREFIXES:
            prefixed_name = f"{prefix}_{joint_name}"
            prefixed_joints[prefixed_name] = dict(limits)
            prefixed_names.append(prefixed_name)
    return prefixed_joints, prefixed_names


def _update_controller_joints(controllers_cfg: dict, joint_names: list) -> dict:
    """Update all controller configs with prefixed joint names."""
    controller_names = controllers_cfg.get("moveit_simple_controller_manager", {}).get(
        "controller_names", []
    )
    for controller_name in controller_names:
        controller_cfg = controllers_cfg.get("moveit_simple_controller_manager", {}).get(
            controller_name
        )
        if isinstance(controller_cfg, dict) and "joints" in controller_cfg:
            controller_cfg["joints"] = joint_names
    return controllers_cfg


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode")
    driver_version = LaunchConfiguration("driver_version")
    namespace = LaunchConfiguration("namespace")
    controller_config_dir = LaunchConfiguration("controller_config_dir")
    rt_core = LaunchConfiguration("rt_core")
    rt_prio = LaunchConfiguration("rt_prio")
    non_rt_cores = LaunchConfiguration("non_rt_cores")
    lock_memory = LaunchConfiguration("lock_memory")

    robot1_model = LaunchConfiguration("robot1_model")
    robot1_family = LaunchConfiguration("robot1_family")
    robot1_prefix = LaunchConfiguration("robot1_prefix")
    robot1_client_ip = LaunchConfiguration("robot1_client_ip")
    robot1_client_port = LaunchConfiguration("robot1_client_port")
    robot1_mxa_client_port = LaunchConfiguration("robot1_mxa_client_port")
    robot1_controller_ip = LaunchConfiguration("robot1_controller_ip")

    robot2_model = LaunchConfiguration("robot2_model")
    robot2_family = LaunchConfiguration("robot2_family")
    robot2_prefix = LaunchConfiguration("robot2_prefix")
    robot2_client_ip = LaunchConfiguration("robot2_client_ip")
    robot2_client_port = LaunchConfiguration("robot2_client_port")
    robot2_mxa_client_port = LaunchConfiguration("robot2_mxa_client_port")
    robot2_controller_ip = LaunchConfiguration("robot2_controller_ip")

    rviz_config_file = (
        get_package_share_directory("kuka_resources") + "/config/planning_6_axis.rviz"
    )

    # Include the dual-arm driver startup launch
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_rsi_driver"), "/launch/dual_arm_startup.launch.py"]
        ),
        launch_arguments=[
            ("mode", mode),
            ("driver_version", driver_version),
            ("namespace", namespace),
            ("controller_config_dir", controller_config_dir),
            ("rt_core", rt_core),
            ("rt_prio", rt_prio),
            ("non_rt_cores", non_rt_cores),
            ("lock_memory", lock_memory),
            ("robot1_model", robot1_model),
            ("robot1_family", robot1_family),
            ("robot1_prefix", robot1_prefix),
            ("robot1_client_ip", robot1_client_ip),
            ("robot1_client_port", robot1_client_port),
            ("robot1_mxa_client_port", robot1_mxa_client_port),
            ("robot1_controller_ip", robot1_controller_ip),
            ("robot2_model", robot2_model),
            ("robot2_family", robot2_family),
            ("robot2_prefix", robot2_prefix),
            ("robot2_client_ip", robot2_client_ip),
            ("robot2_client_port", robot2_client_port),
            ("robot2_mxa_client_port", robot2_mxa_client_port),
            ("robot2_controller_ip", robot2_controller_ip),
        ],
    )

    # Manual MoveIt configuration for multi-robot KR setup.
    # MoveIt config builder cannot be used as it expects URDF and SRDF files
    # to be located in the moveit support package, but this multi-robot setup
    # uses custom SRDF files stored in kuka_multi_robot_examples.
    srdf_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_multi_robot_examples"),
                    "srdf",
                    "multi_kr6_r700_2.srdf",
                ]
            ),
        ],
        on_stderr="capture",
    )

    moveit_config_path = get_package_share_directory("kuka_kr_moveit_config") + "/config"

    # Load and remap kinematics configuration
    with open(moveit_config_path + "/kinematics.yaml") as f:
        kinematics_config = yaml.safe_load(f)
    kinematics_config = _remap_config_for_robots(kinematics_config)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_config}

    # Load MoveIt controllers configuration
    with open(moveit_config_path + "/moveit_controllers.yaml") as f:
        moveit_controllers_config = yaml.safe_load(f)

    # Load and remap OMPL planning configuration
    with open(moveit_config_path + "/ompl_planning.yaml") as f:
        ompl_config = yaml.safe_load(f)
    ompl_pipeline_config = {
        key: value for key, value in ompl_config.items() if key != SINGLE_ARM_GROUP
    }

    # Remap OMPL groups for multi-robot and add projection evaluators.
    # Keep base OMPL pipeline keys (e.g., planning_plugins) intact.
    base_group_cfg = ompl_config.get(SINGLE_ARM_GROUP, {})
    if base_group_cfg:
        for prefix in ROBOT_PREFIXES:
            group_cfg = dict(base_group_cfg)
            group_cfg["projection_evaluator"] = f"joints({prefix}_joint_1,{prefix}_joint_2)"
            ompl_pipeline_config[f"{prefix}_{SINGLE_ARM_GROUP}"] = group_cfg

        # Add dual manipulator group
        dual_cfg = dict(base_group_cfg)
        projection_joints = ",".join(f"{p}_joint_1" for p in ROBOT_PREFIXES)
        dual_cfg["projection_evaluator"] = f"joints({projection_joints})"
        ompl_pipeline_config["dual_manipulator"] = dual_cfg

    # Load Pilz planning configuration
    with open(moveit_config_path + "/pilz_cartesian_limits.yaml") as f:
        pilz_cartesian_limits_config = yaml.safe_load(f)

    with open(moveit_config_path + "/pilz_industrial_motion_planner_planning.yaml") as f:
        pilz_pipeline_config = yaml.safe_load(f)

    # Load and prefix joint limits based on robot1 family/model.
    robot1_family_value = robot1_family.perform(context)
    robot1_model_value = robot1_model.perform(context)
    robot1_support_package = f"kuka_{robot1_family_value}_support"
    joint_limits_file = os.path.join(
        get_package_share_directory(robot1_support_package),
        "config",
        f"{robot1_model_value}_joint_limits.yaml",
    )
    with open(joint_limits_file) as f:
        base_joint_limits_config = yaml.safe_load(f)

    base_joint_limits = base_joint_limits_config.get("joint_limits", {})
    prefixed_joint_limits, prefixed_joint_names = _prefix_joint_names(base_joint_limits)

    # Update controller configurations with prefixed joint names
    moveit_controllers_config = _update_controller_joints(
        moveit_controllers_config, prefixed_joint_names
    )

    # Build planning description
    robot_description_planning = {
        "default_velocity_scaling_factor": base_joint_limits_config.get(
            "default_velocity_scaling_factor", 1.0
        ),
        "default_acceleration_scaling_factor": base_joint_limits_config.get(
            "default_acceleration_scaling_factor", 1.0
        ),
        "joint_limits": prefixed_joint_limits,
        "cartesian_limits": pilz_cartesian_limits_config.get("cartesian_limits", {}),
    }

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description_semantic": ParameterValue(srdf_content, value_type=str)},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            {"planning_pipelines": ["ompl", "pilz_industrial_motion_planner"]},
            {"default_planning_pipeline": "ompl"},
            {"robot_description_planning": robot_description_planning},
            robot_description_kinematics,
            moveit_controllers_config,
            {"ompl": ompl_pipeline_config},
            {"pilz_industrial_motion_planner": pilz_pipeline_config},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            robot_description_kinematics,
        ],
    )

    return [startup_launch, move_group_server, rviz]


def generate_launch_description():
    launch_arguments = []

    launch_arguments.append(DeclareLaunchArgument("mode", default_value="mock"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument(
            "controller_config_dir",
            default_value=get_package_share_directory("kuka_rsi_driver") + "/config",
        )
    )
    launch_arguments.append(DeclareLaunchArgument("rt_core", default_value="-1"))
    launch_arguments.append(DeclareLaunchArgument("rt_prio", default_value="70"))
    launch_arguments.append(DeclareLaunchArgument("non_rt_cores", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("lock_memory", default_value="true"))

    launch_arguments.append(DeclareLaunchArgument("robot1_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot1_family", default_value="agilus"))
    launch_arguments.append(DeclareLaunchArgument("robot1_prefix", default_value="robot1_"))
    launch_arguments.append(DeclareLaunchArgument("robot1_client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot1_client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("robot1_mxa_client_port", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("robot1_controller_ip", default_value="0.0.0.0"))

    launch_arguments.append(DeclareLaunchArgument("robot2_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot2_family", default_value="agilus"))
    launch_arguments.append(DeclareLaunchArgument("robot2_prefix", default_value="robot2_"))
    launch_arguments.append(DeclareLaunchArgument("robot2_client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("robot2_client_port", default_value="59153"))
    launch_arguments.append(DeclareLaunchArgument("robot2_mxa_client_port", default_value="1338"))
    launch_arguments.append(DeclareLaunchArgument("robot2_controller_ip", default_value="0.0.0.0"))

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
