# Copyright 2024 Ádám Pető
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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    ns = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    robot_family_value = robot_family.perform(context)
    robot_model_value = robot_model.perform(context)
    if robot_family_value == "lbr_iisy":
        robot_support_package = "kuka_lbr_iisy_support"
        moveit_config_package = "kuka_lbr_iisy_moveit_config"
        driver_package = "kuka_iiqka_eac_driver"
    else:
        robot_support_package = f"kuka_{robot_family_value}_support"
        moveit_config_package = "kuka_kr_moveit_config"
        driver_package = "kuka_rsi_driver"

    driver_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(driver_package), "/launch/startup.launch.py"]
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(moveit_config_package.removesuffix("_moveit_config"))
        .robot_description(
            file_path=get_package_share_directory(robot_support_package)
            + f"/urdf/{robot_model_value}.urdf.xacro",
            mappings={
                "x": x.perform(context),
                "y": y.perform(context),
                "z": z.perform(context),
                "roll": roll.perform(context),
                "pitch": pitch.perform(context),
                "yaw": yaw.perform(context),
                "prefix": tf_prefix,
            },
        )
        .robot_description_semantic(
            file_path=get_package_share_directory(moveit_config_package)
            + f"/urdf/{robot_model_value}.srdf"
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory(moveit_config_package)
            + "/config/kinematics.yaml"
        )
        .trajectory_execution(
            file_path=get_package_share_directory(moveit_config_package)
            + "/config/moveit_controllers.yaml"
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory(robot_support_package)
            + f"/config/{robot_model_value}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_capabilities],
    )

    # MTC Demo node
    mtc_demo = Node(
        package="moveit_task_constructor_example",
        executable="mtc_depalletizing_example",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    to_start = [driver_startup, move_group_server, mtc_demo]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="lbr_iisy"))
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
