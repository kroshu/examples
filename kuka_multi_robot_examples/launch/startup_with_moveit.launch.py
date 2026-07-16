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
    rviz_config = (
        get_package_share_directory("kuka_resources") + "/config/planning_6_axis.rviz"
    )

    # Include the multi-robot startup launch
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_multi_robot_examples"), "/launch/startup_with_rviz.launch.py"]
        ),
        launch_arguments={"rviz_config": rviz_config}.items(),
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

    with open(moveit_config_path + "/moveit_controllers.yaml", "r") as f:
        moveit_controllers_config = yaml.safe_load(f)

    with open(moveit_config_path + "/ompl_planning.yaml", "r") as f:
        ompl_config = yaml.safe_load(f)

    ompl_pipeline_config = dict(ompl_config)
    planning_plugins = ompl_pipeline_config.get("planning_plugins", [])
    ompl_pipeline_config["planning_plugin"] = (
        planning_plugins[0] if planning_plugins else "ompl_interface/OMPLPlanner"
    )

    with open(moveit_config_path + "/pilz_cartesian_limits.yaml", "r") as f:
        pilz_cartesian_limits_config = yaml.safe_load(f)

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description_semantic": ParameterValue(srdf_content, value_type=str)},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            {"planning_pipelines": ["ompl"]},
            {"default_planning_pipeline": "ompl"},
            kinematics_config,
            moveit_controllers_config,
            {"ompl": ompl_pipeline_config},
            pilz_cartesian_limits_config,
        ],
    )

    return [startup_launch, move_group_server]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
