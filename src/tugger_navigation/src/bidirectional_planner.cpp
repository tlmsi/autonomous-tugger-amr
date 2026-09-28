#include "tugger_navigation/bidirectional_planner.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace tugger_navigation
{

void BidirectionalPlanner::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name,
  nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);

  auto node = parent_.lock();

  if (!node) {
    throw nav2_core::PlannerException(
      "BidirectionalPlanner: parent lifecycle node expired");
  }

  forward_planner_ =
    std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();

  reverse_planner_ =
    std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();

  forward_planner_->configure(
    parent, name_ + ".Forward", tf, costmap_ros);

  reverse_planner_->configure(
    parent, name_ + ".Reverse", tf, costmap_ros);

  locked_direction_ = Direction::NONE;
  have_locked_goal_ = false;
  have_last_call_ = false;

  RCLCPP_INFO(
    node->get_logger(),
    "BidirectionalPlanner configured: FRONT-only and REVERSE-only candidates");
}

void BidirectionalPlanner::cleanup()
{
  if (forward_planner_) {
    forward_planner_->cleanup();
  }

  if (reverse_planner_) {
    reverse_planner_->cleanup();
  }

  forward_planner_.reset();
  reverse_planner_.reset();

  locked_direction_ = Direction::NONE;
  have_locked_goal_ = false;
  have_last_call_ = false;
}

void BidirectionalPlanner::activate()
{
  if (forward_planner_) {
    forward_planner_->activate();
  }

  if (reverse_planner_) {
    reverse_planner_->activate();
  }
}

void BidirectionalPlanner::deactivate()
{
  if (forward_planner_) {
    forward_planner_->deactivate();
  }

  if (reverse_planner_) {
    reverse_planner_->deactivate();
  }

  locked_direction_ = Direction::NONE;
  have_locked_goal_ = false;
  have_last_call_ = false;
}

geometry_msgs::msg::PoseStamped
BidirectionalPlanner::rotateHeadingPi(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped result = pose;

  const double old_z = pose.pose.orientation.z;
  const double old_w = pose.pose.orientation.w;

  result.pose.orientation.x = 0.0;
  result.pose.orientation.y = 0.0;
  result.pose.orientation.z = old_w;
  result.pose.orientation.w = -old_z;

  return result;
}

nav_msgs::msg::Path
BidirectionalPlanner::restoreReverseHeadings(
  const nav_msgs::msg::Path & path) const
{
  nav_msgs::msg::Path result = path;

  for (auto & pose : result.poses) {
    pose = rotateHeadingPi(pose);
  }

  return result;
}

double BidirectionalPlanner::pathLength(
  const nav_msgs::msg::Path & path) const
{
  double length = 0.0;

  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const double dx =
      path.poses[i].pose.position.x -
      path.poses[i - 1].pose.position.x;

    const double dy =
      path.poses[i].pose.position.y -
      path.poses[i - 1].pose.position.y;

    length += std::hypot(dx, dy);
  }

  return length;
}

bool BidirectionalPlanner::sameGoal(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b) const
{
  const double dx =
    a.pose.position.x - b.pose.position.x;

  const double dy =
    a.pose.position.y - b.pose.position.y;

  const double position_difference =
    std::hypot(dx, dy);

  const double dot =
    std::abs(
      a.pose.orientation.x * b.pose.orientation.x +
      a.pose.orientation.y * b.pose.orientation.y +
      a.pose.orientation.z * b.pose.orientation.z +
      a.pose.orientation.w * b.pose.orientation.w);

  return position_difference < 0.05 && dot > 0.999;
}

nav_msgs::msg::Path
BidirectionalPlanner::createForwardPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> cancel_checker)
{
  return forward_planner_->createPlan(
    start,
    goal,
    viapoints,
    cancel_checker);
}

nav_msgs::msg::Path
BidirectionalPlanner::createReversePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> cancel_checker)
{
  const auto reverse_start = rotateHeadingPi(start);
  const auto reverse_goal = rotateHeadingPi(goal);

  std::vector<geometry_msgs::msg::PoseStamped> reverse_viapoints;
  reverse_viapoints.reserve(viapoints.size());

  for (const auto & viapoint : viapoints) {
    reverse_viapoints.push_back(
      rotateHeadingPi(viapoint));
  }

  const auto raw_path =
    reverse_planner_->createPlan(
      reverse_start,
      reverse_goal,
      reverse_viapoints,
      cancel_checker);

  return restoreReverseHeadings(raw_path);
}

nav_msgs::msg::Path
BidirectionalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> cancel_checker)
{
  auto node = parent_.lock();

  if (!node) {
    throw nav2_core::PlannerException(
      "BidirectionalPlanner: parent lifecycle node expired");
  }

  const auto now = std::chrono::steady_clock::now();

  bool new_mission = !have_locked_goal_;

  if (have_locked_goal_ && !sameGoal(goal, locked_goal_)) {
    new_mission = true;
  }

  if (have_last_call_) {
    const double gap =
      std::chrono::duration<double>(
        now - last_plan_call_).count();

    if (gap > 2.5) {
      new_mission = true;
    }
  }

  if (new_mission) {
    locked_direction_ = Direction::NONE;
    locked_goal_ = goal;
    have_locked_goal_ = true;

    RCLCPP_INFO(
      node->get_logger(),
      "New navigation mission: evaluating FRONT and REVERSE");
  }

  /*
   * Once a direction has been selected, replanning is allowed only
   * in that direction. We intentionally do NOT fall back to the
   * opposite direction if the locked planner fails.
   */
  if (locked_direction_ == Direction::FORWARD) {
    try {
      auto path =
        createForwardPlan(
          start, goal, viapoints, cancel_checker);

      last_plan_call_ = std::chrono::steady_clock::now();
      have_last_call_ = true;

      if (path.poses.empty()) {
        throw nav2_core::NoValidPathCouldBeFound(
          "Locked FRONT direction produced an empty path");
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Direction lock: FRONT");

      return path;
    } catch (...) {
      last_plan_call_ = std::chrono::steady_clock::now();
      have_last_call_ = true;
      throw;
    }
  }

  if (locked_direction_ == Direction::REVERSE) {
    try {
      auto path =
        createReversePlan(
          start, goal, viapoints, cancel_checker);

      last_plan_call_ = std::chrono::steady_clock::now();
      have_last_call_ = true;

      if (path.poses.empty()) {
        throw nav2_core::NoValidPathCouldBeFound(
          "Locked REVERSE direction produced an empty path");
      }

      RCLCPP_INFO(
        node->get_logger(),
        "Direction lock: REVERSE");

      return path;
    } catch (...) {
      last_plan_call_ = std::chrono::steady_clock::now();
      have_last_call_ = true;
      throw;
    }
  }

  nav_msgs::msg::Path forward_path;
  nav_msgs::msg::Path reverse_path;

  bool forward_valid = false;
  bool reverse_valid = false;

  try {
    forward_path =
      createForwardPlan(
        start, goal, viapoints, cancel_checker);

    forward_valid = !forward_path.poses.empty();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node->get_logger(),
      "FRONT candidate failed: %s",
      e.what());
  }

  if (cancel_checker && cancel_checker()) {
    throw nav2_core::PlannerCancelled(
      "Bidirectional planning cancelled");
  }

  try {
    reverse_path =
      createReversePlan(
        start, goal, viapoints, cancel_checker);

    reverse_valid = !reverse_path.poses.empty();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node->get_logger(),
      "REVERSE candidate failed: %s",
      e.what());
  }

  last_plan_call_ = std::chrono::steady_clock::now();
  have_last_call_ = true;

  if (!forward_valid && !reverse_valid) {
    throw nav2_core::NoValidPathCouldBeFound(
      "Neither FRONT nor REVERSE candidate produced a valid path");
  }

  const double forward_length =
    forward_valid ?
    pathLength(forward_path) :
    std::numeric_limits<double>::infinity();

  const double reverse_length =
    reverse_valid ?
    pathLength(reverse_path) :
    std::numeric_limits<double>::infinity();

  if (forward_valid &&
      (!reverse_valid || forward_length <= reverse_length))
  {
    locked_direction_ = Direction::FORWARD;

    RCLCPP_INFO(
      node->get_logger(),
      "Mission direction LOCKED: FRONT | front=%.3f m reverse=%.3f m",
      forward_length,
      reverse_length);

    return forward_path;
  }

  locked_direction_ = Direction::REVERSE;

  RCLCPP_INFO(
    node->get_logger(),
    "Mission direction LOCKED: REVERSE | front=%.3f m reverse=%.3f m",
    forward_length,
    reverse_length);

  return reverse_path;
}

}  // namespace tugger_navigation

PLUGINLIB_EXPORT_CLASS(
  tugger_navigation::BidirectionalPlanner,
  nav2_core::GlobalPlanner)
