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

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_example/moveit_example.hpp"
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
constexpr std::array<double, 12> kDualHomeJointPositions = {0.0, -kHalfPi, kHalfPi, 0.0, 0.0, 0.0,
                                                            0.0, -kHalfPi, kHalfPi, 0.0, 0.0, 0.0};

// Convert builtin_interfaces::msg::Duration to seconds as double
double durationToSeconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

// Convert seconds to builtin_interfaces::msg::Duration
builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(seconds);
  d.nanosec = static_cast<uint32_t>((seconds - d.sec) * 1e9);
  return d;
}

// Linear interpolation between two trajectory points at a given time
trajectory_msgs::msg::JointTrajectoryPoint interpolatePoint(
  const trajectory_msgs::msg::JointTrajectoryPoint & p1,
  const trajectory_msgs::msg::JointTrajectoryPoint & p2, double t1, double t2, double t)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  const double alpha = (t2 > t1) ? (t - t1) / (t2 - t1) : 0.0;

  result.positions.resize(p1.positions.size());
  for (size_t i = 0; i < p1.positions.size(); ++i)
  {
    result.positions[i] = p1.positions[i] + alpha * (p2.positions[i] - p1.positions[i]);
  }

  // Interpolate velocities if available
  if (!p1.velocities.empty() && !p2.velocities.empty())
  {
    result.velocities.resize(p1.velocities.size());
    for (size_t i = 0; i < p1.velocities.size(); ++i)
    {
      result.velocities[i] = p1.velocities[i] + alpha * (p2.velocities[i] - p1.velocities[i]);
    }
  }

  // Interpolate accelerations if available
  if (!p1.accelerations.empty() && !p2.accelerations.empty())
  {
    result.accelerations.resize(p1.accelerations.size());
    for (size_t i = 0; i < p1.accelerations.size(); ++i)
    {
      result.accelerations[i] =
        p1.accelerations[i] + alpha * (p2.accelerations[i] - p1.accelerations[i]);
    }
  }

  result.time_from_start = secondsToDuration(t);
  return result;
}

// Sample a trajectory at a specific time, interpolating between points if
// needed
trajectory_msgs::msg::JointTrajectoryPoint sampleTrajectoryAtTime(
  const trajectory_msgs::msg::JointTrajectory & traj, double t)
{
  if (traj.points.empty())
  {
    return {};
  }

  // Find the two points that bracket the requested time
  for (size_t i = 0; i < traj.points.size() - 1; ++i)
  {
    const double t1 = durationToSeconds(traj.points[i].time_from_start);
    const double t2 = durationToSeconds(traj.points[i + 1].time_from_start);

    if (t >= t1 && t <= t2)
    {
      return interpolatePoint(traj.points[i], traj.points[i + 1], t1, t2, t);
    }
  }

  // If past the end, return the last point
  auto last_point = traj.points.back();
  last_point.time_from_start = secondsToDuration(t);
  return last_point;
}

class DualRobotMoveitExample : public MoveitExample
{
public:
  DualRobotMoveitExample() : MoveitExample("moveit_dual_robot_planning_example", "dual_manipulator")
  {
  }

  bool run()
  {
    initialize();
    auto move_group = moveGroupInterface();
    move_group->startStateMonitor(2.0);

    // Try a coordinated Cartesian request first for both TCPs.
    if (!planAndExecutePerArmCartesianPath())
    {
      RCLCPP_ERROR(this->get_logger(), "per-arm Cartesian planning/execution failed, aborting");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "per-arm Cartesian step completed, continuing with joint motion");

    return planAndExecuteJointMotion();
  }

private:
  bool planAndExecuteJointMotion()
  {
    std::vector<double> joint_goal(kDualHomeJointPositions.begin(), kDualHomeJointPositions.end());

    joint_goal[kRobot1Joint1Index] += -0.15;
    joint_goal[kRobot2Joint1Index] += 0.15;

    auto trajectory = planToPosition(joint_goal, "ompl", "RRTConnectkConfigDefault");
    if (!trajectory)
    {
      RCLCPP_ERROR(this->get_logger(), "planning failed for group '%s'", planningGroup().c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "planned dual-arm trajectory with group '%s'", planningGroup().c_str());

    if (!executeTrajectory(*trajectory))
    {
      RCLCPP_ERROR(this->get_logger(), "execution failed for group '%s'", planningGroup().c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "executed dual-arm trajectory with group '%s'", planningGroup().c_str());
    return true;
  }

  bool planAndExecutePerArmCartesianPath()
  {
    moveit::planning_interface::MoveGroupInterface robot1_group(
      shared_from_this(), "robot1_manipulator");
    moveit::planning_interface::MoveGroupInterface robot2_group(
      shared_from_this(), "robot2_manipulator");

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

    // KDL IK solves single chains only, so Cartesian LIN cannot be solved on
    // the composed dual_manipulator group. Plan one trajectory per arm, then
    // merge and execute as one dual-arm goal to keep synchronized start.
    moveit::planning_interface::MoveGroupInterface::Plan robot1_plan;
    moveit::planning_interface::MoveGroupInterface::Plan robot2_plan;
    const bool robot1_planned = static_cast<bool>(robot1_group.plan(robot1_plan));
    const bool robot2_planned = static_cast<bool>(robot2_group.plan(robot2_plan));

    if (!robot1_planned || !robot2_planned)
    {
      RCLCPP_WARN(
        this->get_logger(), "Per-arm LIN planning failed (robot1=%s, robot2=%s)",
        robot1_planned ? "true" : "false", robot2_planned ? "true" : "false");
      return false;
    }

    const auto & jt1 = robot1_plan.trajectory.joint_trajectory;
    const auto & jt2 = robot2_plan.trajectory.joint_trajectory;
    if (jt1.points.empty() || jt2.points.empty())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Per-arm LIN generated empty trajectory (robot1_points=%zu, "
        "robot2_points=%zu)",
        jt1.points.size(), jt2.points.size());
      return false;
    }

    // The two per-arm LIN plans are merged into one combined dual-arm
    // trajectory by resampling both trajectories onto a unified timeline and
    // concatenating the joint values at each time step. This preserves the
    // Cartesian path that was validated by the LIN planner, rather than just
    // interpolating between start and end points in joint space.
    moveit_msgs::msg::RobotTrajectory combined_trajectory;
    auto & combined_jt = combined_trajectory.joint_trajectory;
    combined_jt.header.stamp = jt1.header.stamp;
    combined_jt.joint_names = jt1.joint_names;
    combined_jt.joint_names.insert(
      combined_jt.joint_names.end(), jt2.joint_names.begin(), jt2.joint_names.end());

    // Collect all unique time points from both trajectories
    std::set<double> time_points;
    for (const auto & pt : jt1.points)
    {
      time_points.insert(durationToSeconds(pt.time_from_start));
    }
    for (const auto & pt : jt2.points)
    {
      time_points.insert(durationToSeconds(pt.time_from_start));
    }

    // Sample both trajectories at each time point and combine
    for (double t : time_points)
    {
      auto pt1 = sampleTrajectoryAtTime(jt1, t);
      auto pt2 = sampleTrajectoryAtTime(jt2, t);

      trajectory_msgs::msg::JointTrajectoryPoint combined_pt;
      combined_pt.positions = pt1.positions;
      combined_pt.positions.insert(
        combined_pt.positions.end(), pt2.positions.begin(), pt2.positions.end());

      // Combine velocities if available
      if (!pt1.velocities.empty() && !pt2.velocities.empty())
      {
        combined_pt.velocities = pt1.velocities;
        combined_pt.velocities.insert(
          combined_pt.velocities.end(), pt2.velocities.begin(), pt2.velocities.end());
      }
      else
      {
        combined_pt.velocities.assign(combined_pt.positions.size(), 0.0);
      }

      // Combine accelerations if available
      if (!pt1.accelerations.empty() && !pt2.accelerations.empty())
      {
        combined_pt.accelerations = pt1.accelerations;
        combined_pt.accelerations.insert(
          combined_pt.accelerations.end(), pt2.accelerations.begin(), pt2.accelerations.end());
      }
      else
      {
        combined_pt.accelerations.assign(combined_pt.positions.size(), 0.0);
      }

      combined_pt.time_from_start = secondsToDuration(t);
      combined_jt.points.push_back(std::move(combined_pt));
    }

    if (!executeTrajectory(combined_trajectory))
    {
      RCLCPP_ERROR(this->get_logger(), "Combined Cartesian execution failed");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "planned and executed synchronized "
      "Cartesian trajectory for both robots");
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

  if (success)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Successfully planned and executed one "
      "combined trajectory for both robots.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Combined dual-robot planning or execution failed.");
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
