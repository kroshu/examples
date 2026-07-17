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

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "iiqka_moveit_example/moveit_example.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
class DualRobotMoveitExample : public MoveitExample
{
public:
  DualRobotMoveitExample()
  : MoveitExample("moveit_dual_robot_planning_example", "dual_manipulator")
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  bool run()
  {
    initialize();
    auto move_group = moveGroupInterface();
    move_group->startStateMonitor(2.0);

    auto current_state = move_group->getCurrentState(2.0);
    if (!current_state) {
      RCLCPP_ERROR(
        this->get_logger(),
        "failed to fetch current robot state for group '%s' (check use_sim_time / joint_states timestamps)",
        planningGroup().c_str());
      return false;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      current_state->getJointModelGroup(planningGroup());
    if (!joint_model_group) {
      RCLCPP_ERROR(this->get_logger(), "joint model group '%s' not found", planningGroup().c_str());
      return false;
    }

    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    if (joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "no joint variables available for group '%s'", planningGroup().c_str());
      return false;
    }

    std::map<std::string, double> joint_goal;
    for (const auto & joint_name : joint_names) {
      joint_goal[joint_name] = current_state->getVariablePosition(joint_name);
    }

    if (joint_goal.count("robot1_joint_1") == 0 || joint_goal.count("robot2_joint_1") == 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "group '%s' does not contain both robot1_joint_1 and robot2_joint_1",
        planningGroup().c_str());
      return false;
    }

    joint_goal["robot1_joint_1"] += -0.15;
    joint_goal["robot2_joint_1"] += 0.15;

    auto trajectory = planToJointTargets(joint_goal, "ompl", "RRTConnectkConfigDefault");
    if (!trajectory) {
      RCLCPP_ERROR(this->get_logger(), "planning failed for group '%s'", planningGroup().c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "planned dual-arm trajectory with group '%s'", planningGroup().c_str());

    if (!executeTrajectory(*trajectory)) {
      RCLCPP_ERROR(this->get_logger(), "execution failed for group '%s'", planningGroup().c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "executed dual-arm trajectory with group '%s'", planningGroup().c_str());
    return true;
  }
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualRobotMoveitExample>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  const bool success = node->run();

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      "Successfully planned and executed one combined trajectory for both robots.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Combined dual-robot planning or execution failed.");
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
