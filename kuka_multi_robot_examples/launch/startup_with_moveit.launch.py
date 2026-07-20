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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
    """Prefix joint names for all robots and return dict and flat list.
    
    Returns:
        (prefixed_joint_dict, prefixed_joint_names_list)
    """
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
    rviz_config_file = (
        get_package_share_directory("kuka_resources") + "/config/planning_6_axis.rviz"
    )

    # Include the multi-robot startup launch
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_multi_robot_examples"), "/launch/startup.launch.py"]
        ),
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

    moveit_config_path = (
        get_package_share_directory("kuka_kr_moveit_config") + "/config"
    )

    # Load and remap kinematics configuration
    with open(moveit_config_path + "/kinematics.yaml", "r") as f:
        kinematics_config = yaml.safe_load(f)
    kinematics_config = _remap_config_for_robots(kinematics_config)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_config}

    # Load MoveIt controllers configuration
    with open(moveit_config_path + "/moveit_controllers.yaml", "r") as f:
        moveit_controllers_config = yaml.safe_load(f)

    # Load and remap OMPL planning configuration
    with open(moveit_config_path + "/ompl_planning.yaml", "r") as f:
        ompl_config = yaml.safe_load(f)
    ompl_pipeline_config = dict(ompl_config)

    # Remap OMPL groups for multi-robot and add projection evaluators
    base_group_cfg = ompl_pipeline_config.get(SINGLE_ARM_GROUP, {})
    if base_group_cfg:
        for i, prefix in enumerate(ROBOT_PREFIXES):
            group_cfg = dict(base_group_cfg)
            group_cfg["projection_evaluator"] = f"joints({prefix}_joint_1,{prefix}_joint_2)"
            ompl_pipeline_config[f"{prefix}_{SINGLE_ARM_GROUP}"] = group_cfg
        
        # Add dual manipulator group
        dual_cfg = dict(base_group_cfg)
        dual_cfg["projection_evaluator"] = "joints(robot1_joint_1,robot2_joint_1)"
        ompl_pipeline_config["dual_manipulator"] = dual_cfg

    # Load Pilz planning configuration
    with open(moveit_config_path + "/pilz_cartesian_limits.yaml", "r") as f:
        pilz_cartesian_limits_config = yaml.safe_load(f)

    with open(moveit_config_path + "/pilz_industrial_motion_planner_planning.yaml", "r") as f:
        pilz_pipeline_config = yaml.safe_load(f)

    # Load and prefix joint limits
    with open(
        get_package_share_directory("kuka_agilus_support")
        + "/config/kr6_r700_2_joint_limits.yaml",
        "r",
    ) as f:
        base_joint_limits_config = yaml.safe_load(f)

    base_joint_limits = base_joint_limits_config.get("joint_limits", {})
    prefixed_joint_limits, prefixed_joint_names = _prefix_joint_names(base_joint_limits)

    # Update controller configurations with prefixed joint names
    moveit_controllers_config = _update_controller_joints(moveit_controllers_config, prefixed_joint_names)

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
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
