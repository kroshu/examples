// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <map>
#include <string>
#include <thread>
#include <vector>
#include <cstdint>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
bool plan_for_group(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group_interface,
  const std::string & planning_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  move_group_interface.setPlanningPipelineId("ompl");
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");
  move_group_interface.startStateMonitor(2.0);
  auto current_state = move_group_interface.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(
      node->get_logger(),
      "failed to fetch current robot state for group '%s' (check use_sim_time / joint_states timestamps)",
      planning_group.c_str());
    return false;
  }

  const moveit::core::JointModelGroup * joint_model_group =
    current_state->getJointModelGroup(planning_group);
  if (!joint_model_group) {
    RCLCPP_ERROR(
      node->get_logger(), "joint model group '%s' not found",
      planning_group.c_str());
    return false;
  }

  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  if (joint_names.empty()) {
    RCLCPP_ERROR(
      node->get_logger(), "no joint variables available for group '%s'",
      planning_group.c_str());
    return false;
  }

  std::map<std::string, double> joint_goal;
  for (const auto & joint_name : joint_names) {
    joint_goal[joint_name] = current_state->getVariablePosition(joint_name);
  }

  if (joint_goal.count("robot1_joint_1") == 0 || joint_goal.count("robot2_joint_1") == 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "group '%s' does not contain both robot1_joint_1 and robot2_joint_1",
      planning_group.c_str());
    return false;
  }

  joint_goal["robot1_joint_1"] += -0.15;
  joint_goal["robot2_joint_1"] += 0.15;

  move_group_interface.setStartState(*current_state);
  move_group_interface.setJointValueTarget(joint_goal);

  const bool planned = static_cast<bool>(move_group_interface.plan(plan));

  if (planned) {
    RCLCPP_INFO(
      node->get_logger(), "planned dual-arm trajectory with group '%s'",
      planning_group.c_str());
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "planning failed for group '%s'",
      planning_group.c_str());
  }

  return planned;
}

bool enforce_synchronized_joint_timing(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const double duration_seconds = 5.0,
  const std::size_t interpolation_steps = 100)
{
  auto & jt = plan.trajectory.joint_trajectory;
  if (jt.points.empty()) {
    RCLCPP_ERROR(node->get_logger(), "planned trajectory is empty");
    return false;
  }
  if (jt.points.size() < 2) {
    RCLCPP_WARN(node->get_logger(), "planned trajectory has a single point, skipping synchronization");
    return true;
  }

  const std::vector<double> start_positions = jt.points.front().positions;
  const std::vector<double> goal_positions = jt.points.back().positions;
  if (start_positions.size() != goal_positions.size()) {
    RCLCPP_ERROR(node->get_logger(), "start/goal trajectory point size mismatch");
    return false;
  }

  jt.points.clear();
  jt.points.reserve(interpolation_steps + 1);

  for (std::size_t i = 0; i <= interpolation_steps; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(interpolation_steps);

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(start_positions.size());
    p.velocities.resize(start_positions.size(), 0.0);
    p.accelerations.resize(start_positions.size(), 0.0);

    for (std::size_t j = 0; j < start_positions.size(); ++j) {
      p.positions[j] = start_positions[j] + alpha * (goal_positions[j] - start_positions[j]);
    }

    const double t = alpha * duration_seconds;
    p.time_from_start.sec = static_cast<int32_t>(t);
    p.time_from_start.nanosec =
      static_cast<uint32_t>((t - static_cast<double>(p.time_from_start.sec)) * 1e9);
    jt.points.push_back(std::move(p));
  }

  RCLCPP_INFO(
    node->get_logger(),
    "re-timed dual-arm plan for synchronized execution (duration=%.2fs, points=%zu)",
    duration_seconds, jt.points.size());
  return true;
}

bool execute_for_group(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group_interface,
  const std::string & planning_group,
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const bool executed = static_cast<bool>(move_group_interface.execute(plan));
  if (!executed) {
    RCLCPP_ERROR(
      node->get_logger(), "execution failed for group '%s'",
      planning_group.c_str());
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(), "executed dual-arm trajectory with group '%s'",
    planning_group.c_str());
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("moveit_dual_robot_planning_example", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  const std::string dual_group = "dual_manipulator";
  moveit::planning_interface::MoveGroupInterface dual_move_group_interface(node, dual_group);
  moveit::planning_interface::MoveGroupInterface::Plan dual_plan;

  const bool dual_plan_ok = plan_for_group(node, dual_move_group_interface, dual_group, dual_plan);
  bool dual_sync_ok = false;
  if (dual_plan_ok) {
    dual_sync_ok = enforce_synchronized_joint_timing(node, dual_plan);
  }

  bool dual_exec_ok = false;
  if (dual_plan_ok && dual_sync_ok) {
    dual_exec_ok = execute_for_group(node, dual_move_group_interface, dual_group, dual_plan);
  }

  if (dual_plan_ok && dual_sync_ok && dual_exec_ok) {
    RCLCPP_INFO(
      node->get_logger(),
      "Successfully planned and executed one combined trajectory for both robots.");
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "Combined dual-robot planning or execution failed.");
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return (dual_plan_ok && dual_sync_ok && dual_exec_ok) ? 0 : 1;
}
