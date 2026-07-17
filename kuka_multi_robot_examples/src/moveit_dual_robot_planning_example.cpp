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

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "iiqka_moveit_example/moveit_example.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kCartesianXOffset = 0.10;
constexpr double kCartesianYOffset = 0.0;
constexpr double kCartesianZOffset = 0.0;
constexpr double kHalfPi = 1.5707963267948966;
constexpr std::size_t kRobot1Joint1Index = 0;
constexpr std::size_t kRobot2Joint1Index = 6;
constexpr std::array<double, 12> kDualHomeJointPositions = {
  0.0, -kHalfPi, kHalfPi, 0.0, 0.0, 0.0,
  0.0, -kHalfPi, kHalfPi, 0.0, 0.0, 0.0};

class DualRobotMoveitExample : public MoveitExample
{
public:
  DualRobotMoveitExample()
  : MoveitExample("moveit_dual_robot_planning_example", "dual_manipulator")
  {}

  bool run()
  {
    initialize();
    auto move_group = moveGroupInterface();
    move_group->startStateMonitor(2.0);

    // Try a coordinated Cartesian request first for both TCPs.
    if (!planAndExecutePerArmCartesianPath()) {
        RCLCPP_ERROR(this->get_logger(), "per-arm Cartesian planning/execution failed, aborting");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "per-arm Cartesian step completed, continuing with joint motion");

    return planAndExecuteJointMotion();
  }

private:
  bool planAndExecuteJointMotion()
  {
    std::vector<double> joint_goal(
      kDualHomeJointPositions.begin(), kDualHomeJointPositions.end());

    joint_goal[kRobot1Joint1Index] += -0.15;
    joint_goal[kRobot2Joint1Index] += 0.15;

    auto trajectory = planToPosition(joint_goal, "ompl", "RRTConnectkConfigDefault");
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

  bool planAndExecutePerArmCartesianPath()
  {
    moveit::planning_interface::MoveGroupInterface robot1_group(shared_from_this(), "robot1_manipulator");
    moveit::planning_interface::MoveGroupInterface robot2_group(shared_from_this(), "robot2_manipulator");

    robot1_group.startStateMonitor(2.0);
    robot2_group.startStateMonitor(2.0);
    robot1_group.setStartStateToCurrentState();
    robot2_group.setStartStateToCurrentState();

    const auto robot1_current = robot1_group.getCurrentPose("robot1_tool0").pose;
    const auto robot2_current = robot2_group.getCurrentPose("robot2_tool0").pose;

    geometry_msgs::msg::Pose robot1_target;
    robot1_target = robot1_current;
    robot1_target.position.x += kCartesianXOffset;
    robot1_target.position.y += kCartesianYOffset;
    robot1_target.position.z += kCartesianZOffset;

    geometry_msgs::msg::Pose robot2_target;
    robot2_target = robot2_current;
    robot2_target.position.x += kCartesianXOffset;
    robot2_target.position.y += kCartesianYOffset;
    robot2_target.position.z += kCartesianZOffset;

    robot1_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    robot1_group.setPlannerId("LIN");
    robot2_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    robot2_group.setPlannerId("LIN");

    robot1_group.setPoseTarget(robot1_target, "robot1_tool0");
    robot2_group.setPoseTarget(robot2_target, "robot2_tool0");

    // KDL IK solves single chains only, so Cartesian LIN cannot be solved on the
    // composed dual_manipulator group. Plan one trajectory per arm, then merge
    // and execute as one dual-arm goal to keep synchronized start.
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan;
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan;
    const bool robot1_planned = static_cast<bool>(robot1_group.plan(robot1_plan));
    const bool robot2_planned = static_cast<bool>(robot2_group.plan(robot2_plan));

    if (!robot1_planned || !robot2_planned) {
      RCLCPP_WARN(
        this->get_logger(),
        "Per-arm LIN planning failed (robot1=%s, robot2=%s)",
        robot1_planned ? "true" : "false",
        robot2_planned ? "true" : "false");
      return false;
    }

    const auto & jt1 = robot1_plan.trajectory.joint_trajectory;
    const auto & jt2 = robot2_plan.trajectory.joint_trajectory;
    if (jt1.points.empty() || jt2.points.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Per-arm LIN generated empty trajectory (robot1_points=%zu, robot2_points=%zu)",
        jt1.points.size(),
        jt2.points.size());
      return false;
    }

    moveit_msgs::msg::RobotTrajectory combined_trajectory;
    auto & combined_jt = combined_trajectory.joint_trajectory;
    combined_jt.header.stamp = jt1.header.stamp;
    combined_jt.joint_names = jt1.joint_names;
    combined_jt.joint_names.insert(
      combined_jt.joint_names.end(),
      jt2.joint_names.begin(),
      jt2.joint_names.end());

    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = jt1.points.front().positions;
    start_point.positions.insert(
      start_point.positions.end(),
      jt2.points.front().positions.begin(),
      jt2.points.front().positions.end());
    start_point.velocities.assign(start_point.positions.size(), 0.0);
    start_point.accelerations.assign(start_point.positions.size(), 0.0);

    trajectory_msgs::msg::JointTrajectoryPoint goal_point;
    goal_point.positions = jt1.points.back().positions;
    goal_point.positions.insert(
      goal_point.positions.end(),
      jt2.points.back().positions.begin(),
      jt2.points.back().positions.end());
    goal_point.velocities.assign(goal_point.positions.size(), 0.0);
    goal_point.accelerations.assign(goal_point.positions.size(), 0.0);

    const auto t1 = jt1.points.back().time_from_start;
    const auto t2 = jt2.points.back().time_from_start;
    goal_point.time_from_start = (t2.sec > t1.sec || (t2.sec == t1.sec && t2.nanosec > t1.nanosec)) ? t2 : t1;

    combined_jt.points.push_back(std::move(start_point));
    combined_jt.points.push_back(std::move(goal_point));

    if (!executeTrajectory(combined_trajectory)) {
      RCLCPP_ERROR(this->get_logger(), "Combined Cartesian execution failed");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "planned and executed synchronized Cartesian trajectory for both robots");
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
