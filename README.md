# Examples

This repository contains examples of how the KUKA drivers can be integrated into the ROS 2 ecosystem for more complex applications needing path planning, I/O-s, additional tools or external axes. For more information on how the KUKA drivers work, visit the [`kuka_drivers` Wiki](https://github.com/kroshu/kuka_drivers/wiki).

## iiQKA MoveIt Example

The `iiqka_moveit_example` package demonstrates integration of the KUKA iiQKA driver with MoveIt 2 for motion planning and manipulation tasks.

**Features:**
- Motion planning using MoveIt 2's planning algorithms (RRT, PRM, etc.)
- Real-time trajectory execution on LBR iisy cobots
- Support for multiple control modes (position, impedance, torque)
- Visualization with RViz and MoveIt 2 plugin
- Example trajectory publisher and motion planning executors

**Startup:**
```bash
ros2 launch iiqka_moveit_example moveit_planning_example.launch.py
```

## Multi-Robot Support

The `kuka_multi_robot_examples` package demonstrates running multiple KUKA robots (both iiQKA and industrial) within a single ROS 2 control node using asynchronous hardware interfaces.

**Key Features:**
- Multiple robots in one `controller_manager` (ROS 2 Jazzy+)
- Asynchronous hardware interfaces running in separate execution contexts
- Independent robot prefixes and namespaces
- Configurable async thread priority and CPU affinity for real-time optimization
- Both synchronous and asynchronous robots can run together (robot1 async, robot2 sync by default)

**Architecture:**
- Single combined URDF with both robot models
- Unified robot description and controller configuration
- Synchronized lifecycle interface for safe multi-robot startup

**Launch files:**
- `startup.launch.py`: Core multi-robot launcher (all features, configurable)
- `startup_with_rviz.launch.py`: Startup with RViz visualization
- `startup_with_moveit.launch.py`: Startup with MoveIt 2 motion planning

**Available launch arguments:**

*Mode & Driver Configuration:*
- `mode` (default: `hardware`): `mock`, `hardware`, or `gazebo`
- `driver_version` (default: `rsi_only`): `rsi_only`, `eki_rsi`, or `mxa_rsi`
- `controller_config` (default: empty): Custom controller configuration file

*Robot 1:*
- `prefix` (default: `robot1_`): Namespace prefix for robot1 resources
- `client_ip` (default: `0.0.0.0`): Client IP address for communication
- `client_port` (default: `59152`): RSI communication port
- `mxa_client_port` (default: `1337`): MXA communication port
- `controller_ip` (default: `0.0.0.0`): Robot controller IP
- `use_gpio` (default: `false`): Enable GPIO I/O support

*Robot 2:*
- `prefix2` (default: `robot2_`): Namespace prefix for robot2 resources
- `client_ip2` (default: `0.0.0.0`): Client IP address for communication
- `client_port2` (default: `59152`): RSI communication port
- `mxa_client_port2` (default: `1337`): MXA communication port
- `controller_ip2` (default: `0.0.0.0`): Robot controller IP
- `use_gpio2` (default: `false`): Enable GPIO I/O support

*Async Hardware Interface (Real-Time Optimization):*
- `async_thread_priority` (default: `69`): Priority for async hardware thread (0-99, higher = more priority)
- `async_affinity` (default: `[]`): CPU core pinning for async thread, e.g. `"[2,4]"` or `"[2]"` (empty `[]` = any core)
- `roundtrip_time` (default: `4000`): Microseconds for RSI communication roundtrip (mock hardware only)

*System & Real-Time Configuration:*
- `namespace` (default: empty): ROS 2 namespace for all nodes
- `verify_robot_model` (default: `true`): Verify robot URDF model validity
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
