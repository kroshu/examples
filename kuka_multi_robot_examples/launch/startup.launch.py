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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode")
    driver_version = LaunchConfiguration("driver_version")
    controller_config = LaunchConfiguration("controller_config")
    roundtrip_time = LaunchConfiguration("roundtrip_time")
    verify_robot_model = LaunchConfiguration("verify_robot_model")
    ns = LaunchConfiguration("namespace")
    non_rt_cores = LaunchConfiguration("non_rt_cores")
    rt_core = LaunchConfiguration("rt_core")
    rt_prio = LaunchConfiguration("rt_prio")
    lock_memory = LaunchConfiguration("lock_memory")
    async_thread_priority = LaunchConfiguration("async_thread_priority")
    async_affinity = LaunchConfiguration("async_affinity")
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    # Parse allowed cores into a list of integers; allow formats like "2,3, 4" or "  "
    cores = []
    for part in non_rt_cores.perform(context).split(","):
        part = part.strip()
        if part == "":
            continue
        try:
            cores.append(int(part))
        except ValueError:
            raise RuntimeError(
                f"Invalid allowed_cores entry: '{part}'. "
                "Provide a comma-separated list of integers, e.g. '2,3,4'."
            )

    # Compute the prefix: None if no cores; otherwise build 'taskset -c <list>'
    prefix_cmd = None
    if cores:
        # Build the string "2,3,4" for taskset
        core_list_str = ",".join(str(c) for c in cores)
        prefix_cmd = f"taskset -c {core_list_str}"

    if not controller_config.perform(context):
        rel_path_to_config_file = (
            "/config/ros2_controller_config_rsi_only.yaml"
            if driver_version.perform(context) == "rsi_only"
            else "/config/ros2_controller_config_extended.yaml"
        )
        controller_config = (
            get_package_share_directory("kuka_rsi_driver") + rel_path_to_config_file
        )

    # Get URDF via xacro
    prefix = LaunchConfiguration("prefix")
    prefix2 = LaunchConfiguration("prefix2")
    client_ip = LaunchConfiguration("client_ip")
    client_port = LaunchConfiguration("client_port")
    mxa_client_port = LaunchConfiguration("mxa_client_port")
    controller_ip = LaunchConfiguration("controller_ip")
    use_gpio = LaunchConfiguration("use_gpio")
    use_gpio2 = LaunchConfiguration("use_gpio2")
    client_ip2 = LaunchConfiguration("client_ip2")
    client_port2 = LaunchConfiguration("client_port2")
    mxa_client_port2 = LaunchConfiguration("mxa_client_port2")
    controller_ip2 = LaunchConfiguration("controller_ip2")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kuka_multi_robot_examples"),
                    "urdf",
                    "multi_kr6_r700_2.urdf.xacro",
                ]
            ),
            " ",
            "mode:=",
            mode,
            " ",
            "driver_version:=",
            driver_version,
            " ",
            "prefix:=",
            prefix,
            " ",
            "client_ip:=",
            client_ip,
            " ",
            "client_port:=",
            client_port,
            " ",
            "mxa_client_port:=",
            mxa_client_port,
            " ",
            "controller_ip:=",
            controller_ip,
            " ",
            "use_gpio:=",
            use_gpio,
            " ",
            "roundtrip_time:=",
            roundtrip_time,
            " ",
            "prefix2:=",
            prefix2,
            " ",
            "client_ip2:=",
            client_ip2,
            " ",
            "client_port2:=",
            client_port2,
            " ",
            "mxa_client_port2:=",
            mxa_client_port2,
            " ",
            "controller_ip2:=",
            controller_ip2,
            " ",
            "use_gpio2:=",
            use_gpio2,
            " ",
            "verify_robot_model:=",
            verify_robot_model,
            " ",
            "async_thread_priority:=",
            async_thread_priority,
            " ",
            "async_affinity:=",
            async_affinity,
        ],
        on_stderr="capture",
    )

    robot_description = {"robot_description": robot_description_content}

    # The driver config contains only parameters that can be changed after startup
    driver_config = get_package_share_directory("kuka_rsi_driver") + "/config/driver_config.yaml"

    control_node = Node(
        namespace=ns,
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            robot_description,
            controller_config,
            {
                "cpu_affinity": int(rt_core.perform(context)),
                "thread_priority": int(rt_prio.perform(context)),
                "lock_memory": lock_memory.perform(context) == "true",
                "hardware_components_initial_state": {
                    "unconfigured": ["robot1_kr6_r700_2", "robot2_kr6_r700_2"],
                },
            },
        ],
        prefix=prefix_cmd,
    )
    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace=ns,
        package="kuka_rsi_driver",
        executable=(
            "robot_manager_node_rsi_only"
            if driver_version.perform(context) == "rsi_only"
            else "robot_manager_node_extended"
        ),
        parameters=[
            driver_config,
            {
                "robot_models": ["robot1_kr6_r700_2", "robot2_kr6_r700_2"],
                "use_gpio": use_gpio,
            },
        ],
        prefix=prefix_cmd,
    )
    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        prefix=prefix_cmd,
    )

    # Spawn controllers
    def controller_spawner(controller_name, prefix_cmd, param_file=None, activate=False):
        arg_list = [
            controller_name,
            "-c",
            "controller_manager",
            "-n",
            ns,
        ]

        # Add param-file if it's provided
        if param_file:
            arg_list.extend(["--param-file", param_file])

        if not activate:
            arg_list.append("--inactive")

        return Node(
            package="controller_manager",
            executable="spawner",
            prefix=prefix_cmd,
            arguments=arg_list,
        )

    controllers = {
        "joint_state_broadcaster": None,
        "joint_trajectory_controller": get_package_share_directory("kuka_multi_robot_examples") + "/config/joint_trajectory_controller_config.yaml",
        "event_broadcaster": get_package_share_directory("kuka_multi_robot_examples") + "/config/kuka_event_broadcaster_config.yaml",
    }

    if use_gpio.perform(context) == "true":
        controllers["gpio_controller"] = gpio_config

    if driver_version.perform(context) in {"eki_rsi", "mxa_rsi"}:
        controllers["control_mode_handler"] = get_package_share_directory(
            "kuka_multi_robot_examples"
        ) + "/config/kuka_control_mode_handler_config.yaml"
        controllers["kss_message_handler"] = get_package_share_directory(
            "kuka_multi_robot_examples"
        ) + "/config/kuka_kss_message_handler_config.yaml"

    controller_spawners = [
        controller_spawner(name, prefix_cmd, param_file)
        for name, param_file in controllers.items()
    ]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("mode", default_value="hardware"))
    launch_arguments.append(DeclareLaunchArgument("controller_config", default_value=""))
    launch_arguments.append(
        DeclareLaunchArgument(
            "driver_version",
            default_value="rsi_only",
            description="Select the driver version to use",
            choices=["rsi_only", "eki_rsi", "mxa_rsi"],
        )
    )
    
    # Robot 1 arguments
    launch_arguments.append(DeclareLaunchArgument("prefix", default_value="robot1_"))
    launch_arguments.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("client_port", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("mxa_client_port", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("controller_ip", default_value="0.0.0.0"))
    launch_arguments.append(
        DeclareLaunchArgument("use_gpio", default_value="false", choices=["true", "false"])
    )
    
    # Robot 2 arguments
    launch_arguments.append(DeclareLaunchArgument("prefix2", default_value="robot2_"))
    launch_arguments.append(DeclareLaunchArgument("client_ip2", default_value="0.0.0.0"))
    launch_arguments.append(DeclareLaunchArgument("client_port2", default_value="59152"))
    launch_arguments.append(DeclareLaunchArgument("mxa_client_port2", default_value="1337"))
    launch_arguments.append(DeclareLaunchArgument("controller_ip2", default_value="0.0.0.0"))
    launch_arguments.append(
        DeclareLaunchArgument("use_gpio2", default_value="false", choices=["true", "false"])
    )
    
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("async_thread_priority", default_value="69"))
    launch_arguments.append(DeclareLaunchArgument("async_affinity", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("roundtrip_time", default_value="4000"))
    launch_arguments.append(
        DeclareLaunchArgument(
            "verify_robot_model", default_value="true", choices=["true", "false"]
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rt_core",
            default_value="-1",  # -1 means do not pin to core
            description=("CPU core index for taskset pinning of the RT thread"),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "rt_prio",
            default_value="70",
            description=("The priority of the thread that runs the control loop"),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "non_rt_cores",
            default_value="",
            description=(
                "Comma-separated CPU core indices for taskset pinning of non-RT threads "
                "(e.g. '2,3,4'). Leave empty to disable pinning."
            ),
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            "lock_memory",
            default_value="true",
            description=(
                "Whether to lock memory of the control loop with mlockall to avoid paging"
            ),
        )
    )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
