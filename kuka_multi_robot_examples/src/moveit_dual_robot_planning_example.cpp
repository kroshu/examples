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

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

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
  bool dual_exec_ok = false;
  if (dual_plan_ok) {
    dual_exec_ok = execute_for_group(node, dual_move_group_interface, dual_group, dual_plan);
  }

  if (dual_plan_ok && dual_exec_ok) {
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
  return (dual_plan_ok && dual_exec_ok) ? 0 : 1;
}
