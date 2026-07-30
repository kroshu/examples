# Copyright 2022 Áron Svastits
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")

    robot_family_value = robot_family.perform(context)
    robot_model_value = robot_model.perform(context)

    if robot_family_value == "lbr_iisy":
        moveit_config_pkg = "kuka_lbr_iisy_moveit_config"
        driver_pkg = "kuka_iiqka_eac_driver"
    elif robot_family_value == "lbr_iiwa":
        moveit_config_pkg = "kuka_lbr_iiwa_moveit_config"
        driver_pkg = "kuka_sunrise_fri_driver"
    else:
        moveit_config_pkg = "kuka_kr_moveit_config"
        driver_pkg = "kuka_rsi_driver"

    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(driver_pkg), "/launch/startup.launch.py"]
        ),
    )

    moveit_server_args = {"robot_model": robot_model_value}
    if robot_family_value not in ("lbr_iisy", "lbr_iiwa"):
        moveit_server_args["robot_family"] = robot_family_value

    moveit_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(moveit_config_pkg), "/launch/moveit_server.launch.py"]
        ),
        launch_arguments=moveit_server_args.items(),
    )

    return [startup_launch, moveit_server]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="lbr_iisy"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
