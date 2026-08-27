# Examples

This repository contains examples of how the KUKA drivers can be integrated into the ROS 2 ecosystem for more complex applications needing path planning, I/O-s, additional tools or external axes. For more information on how the KUKA drivers work, visit the [`kuka_drivers` Wiki](https://github.com/kroshu/kuka_drivers/wiki).

## MoveIt Example

The `moveit_example` package demonstrates integration of the KUKA drivers with MoveIt 2 for motion planning and manipulation tasks.

**Features:**
- Motion planning using MoveIt 2's planning algorithms (RRT, PRM, etc.)
- Real-time trajectory execution on LBR iisy cobots
- Support for multiple control modes (position, impedance, torque)
- Visualization with RViz and MoveIt 2 plugin
- Example trajectory publisher and motion planning executors

**Startup:**
```bash
ros2 launch moveit_example moveit_planning_example.launch.py
```

## MoveIt Task Constructor Example

The `moveit_task_constructor_example` package demonstrates a depalletizing task using the MoveIt Task Constructor with KUKA LBR iisy robots.

**Features:**
- Plans and executes a pick-and-place task for a 2x2x2 pallet arrangement
- Uses OMPL, joint interpolation, and Cartesian path planners
- Visualizes the planning scene and task solutions in RViz

**Startup:**
```bash
ros2 launch moveit_task_constructor_example startup_with_rviz.launch.py
```

To run without RViz:
```bash
ros2 launch moveit_task_constructor_example startup.launch.py
```

**Available launch arguments:**
- `robot_model` (default: `lbr_iisy3_r760`): Robot model name
- `robot_family` (default: `lbr_iisy`): Robot family; use an RSI-based industrial family such as `agilus` with its matching model
- `namespace` (default: empty): ROS 2 namespace for all nodes
- `x`, `y`, `z` (default: `0`): Position of `base_link` relative to `world` in meters
- `roll`, `pitch`, `yaw` (default: `0`): Orientation of `base_link` relative to `world` in radians

For example, to use a KR Agilus robot:
```bash
ros2 launch moveit_task_constructor_example startup_with_rviz.launch.py \
    robot_family:=agilus robot_model:=kr6_r700_sixx
```

Sunrise LBR iiwa robots are not supported by this example.

## Multi-Robot Support

The `kuka_multi_robot_examples` package demonstrates planning with multiple KUKA robots (both iiQKA and industrial) within a single ROS 2 control node using asynchronous hardware interfaces using MoveIt.


**Launch file:**
- `startup_with_moveit.launch.py`: Startup with MoveIt 2 motion planning
    - MoveitConfigsBuilder cannot be used here, therefore all configurations have to be loaded manually. The launch files only loads ompl and pilz planning configurations.

**Available launch arguments:**

*Mode & Driver Configuration:*
- `mode` (default: `mock`): `mock`, `hardware`, or `gazebo`
- `driver_version` (default: `rsi_only`): `rsi_only`, `eki_rsi`, or `mxa_rsi`
- `controller_config_dir` (default: `kuka_rsi_driver/config`): Controller configuration directory

*Robot 1:*
- `robot1_model` (default: `kr6_r700_sixx`): Robot model name
- `robot1_family` (default: `agilus`): Robot family name
- `robot1_prefix` (default: `robot1_`): Namespace prefix for robot1 resources
- `robot1_client_ip` (default: `0.0.0.0`): Client IP address for communication
- `robot1_client_port` (default: `59152`): RSI communication port
- `robot1_mxa_client_port` (default: `1337`): MXA communication port
- `robot1_controller_ip` (default: `0.0.0.0`): Robot controller IP

*Robot 2:*
- `robot2_model` (default: `kr6_r700_sixx`): Robot model name
- `robot2_family` (default: `agilus`): Robot family name
- `robot2_prefix` (default: `robot2_`): Namespace prefix for robot2 resources
- `robot2_client_ip` (default: `0.0.0.0`): Client IP address for communication
- `robot2_client_port` (default: `59153`): RSI communication port
- `robot2_mxa_client_port` (default: `1338`): MXA communication port
- `robot2_controller_ip` (default: `0.0.0.0`): Robot controller IP

*System & Real-Time Configuration:*
- `namespace` (default: empty): ROS 2 namespace for all nodes
- `rt_core` (default: `-1`): CPU core for real-time control loop pinning (-1 = no pinning)
- `rt_prio` (default: `70`): Priority level for control loop thread
- `non_rt_cores` (default: empty): Comma-separated CPU cores for non-RT threads, e.g. `"2,3,4"`
- `lock_memory` (default: `true`): Lock control loop memory to prevent paging

**Startup**
```bash
# Basic multi-robot with mock hardware
ros2 launch kuka_multi_robot_examples startup_with_rviz.launch.py mode:=mock
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```
