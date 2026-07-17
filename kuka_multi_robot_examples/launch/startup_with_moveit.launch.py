# Copyright 2023 KUKA Hungaria Kft.
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

    # Manual MoveIt configuration for multi-robot KR setup
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

    with open(moveit_config_path + "/kinematics.yaml", "r") as f:
        kinematics_config = yaml.safe_load(f)

    # Remap base single-arm kinematics config to prefixed multi-robot group names.
    base_kinematics_cfg = kinematics_config.get("manipulator", {})
    if base_kinematics_cfg:
        kinematics_config = {
            "robot1_manipulator": dict(base_kinematics_cfg),
            "robot2_manipulator": dict(base_kinematics_cfg),
        }
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_config
    }

    with open(moveit_config_path + "/moveit_controllers.yaml", "r") as f:
        moveit_controllers_config = yaml.safe_load(f)

    with open(moveit_config_path + "/ompl_planning.yaml", "r") as f:
        ompl_config = yaml.safe_load(f)

    ompl_pipeline_config = dict(ompl_config)
    planning_plugins = ompl_pipeline_config.get("planning_plugins", [])
    ompl_pipeline_config["planning_plugin"] = (
        planning_plugins[0] if planning_plugins else "ompl_interface/OMPLPlanner"
    )
    base_group_cfg = ompl_pipeline_config.get("manipulator", {})
    if base_group_cfg:
        robot1_group_cfg = dict(base_group_cfg)
        robot1_group_cfg["projection_evaluator"] = "joints(robot1_joint_1,robot1_joint_2)"

        robot2_group_cfg = dict(base_group_cfg)
        robot2_group_cfg["projection_evaluator"] = "joints(robot2_joint_1,robot2_joint_2)"

        dual_group_cfg = dict(base_group_cfg)
        dual_group_cfg["projection_evaluator"] = "joints(robot1_joint_1,robot2_joint_1)"

        ompl_pipeline_config["robot1_manipulator"] = robot1_group_cfg
        ompl_pipeline_config["robot2_manipulator"] = robot2_group_cfg
        ompl_pipeline_config["dual_manipulator"] = dual_group_cfg

    with open(moveit_config_path + "/pilz_cartesian_limits.yaml", "r") as f:
        pilz_cartesian_limits_config = yaml.safe_load(f)

    pilz_pipeline_config = {
        "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
        "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "start_state_max_bounds_error": 0.1,
    }

    with open(
        get_package_share_directory("kuka_agilus_support")
        + "/config/kr6_r700_2_joint_limits.yaml",
        "r",
    ) as f:
        base_joint_limits_config = yaml.safe_load(f)

    base_joint_limits = base_joint_limits_config.get("joint_limits", {})
    prefixed_joint_limits = {}
    prefixed_joint_names = []
    for joint_name, limits in base_joint_limits.items():
        prefixed_joint_limits[f"robot1_{joint_name}"] = dict(limits)
        prefixed_joint_limits[f"robot2_{joint_name}"] = dict(limits)
        prefixed_joint_names.append(f"robot1_{joint_name}")
        prefixed_joint_names.append(f"robot2_{joint_name}")

    controller_names = moveit_controllers_config.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller_name in controller_names:
        controller_cfg = moveit_controllers_config.get(
            "moveit_simple_controller_manager", {}
        ).get(controller_name)
        if isinstance(controller_cfg, dict) and "joints" in controller_cfg:
            controller_cfg["joints"] = prefixed_joint_names

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
