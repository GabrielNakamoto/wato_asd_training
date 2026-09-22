#include "planner_node.hpp"
#include "a_star.hpp"
#include <queue>
#include <chrono>

PlannerNode::PlannerNode() : Node("planner"), state_(State::NO_GOAL) {
  auto odomCallback = [&](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    robot_pose_ = msg->pose.pose;
  };

  auto goalCallback = [&](const std::shared_ptr<geometry_msgs::msg::PointStamped> msg) {
    goal_ = *msg;
    state_ = State::PLANNING;
    computePath();
  };

  auto mapCallback = [&](const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg) {
    current_map_ = *msg;
    computePath();
  };

  auto timerCallback = [&] {
    if (state_ != State::PLANNING) return;
    double delta = std::hypot(goal_.point.x - robot_pose_.position.x, goal_.point.y - robot_pose_.position.y);

    if (delta < PlannerNode::completed_threshold) {
      RCLCPP_INFO(this->get_logger(), "GOAL reached!");
      state_ = State::NO_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "replanning...");
      computePath();
    }
  };

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, goalCallback);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, odomCallback);
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, mapCallback);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timerCallback);
}

CellIndex PlannerNode::worldToCell(double wx, double wy) {
  const auto &map_info = current_map_->info
  double gx = std::floor((wx + map_info.origin.position.x) / map_info.resolution);
  double cy = std::floor((wy + map_info.origin.position.y) / map_info.resolution);
  return CellIndex(gx, gy);
}

void PlannerNode::computePath() {
  if (state_ != State::PLANNING || current_map_.data.empty()) return;

  const CellIndex current_cell = worldToCell(robot_pose_.position.x, robot_pose_.position.y);
  const CellIndex target_cell = worldToCell(goal_.point.x, goal_.point.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> queue;
  std::unordered_map<> g_score;

  // queue.emplace(current_cell, 

  // A* implementation
  while (! queue.empty()) {
    AStarNode node = queue.top();
    queue.pop();

  }

  auto path = nav_msgs::msg::Path();
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
