#include "planner_node.hpp"
#include <algorithm>
#include <queue>
#include <optional>
#include <chrono>

PlannerNode::PlannerNode() : Node("planner"), state_(State::NO_GOAL) {
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ robot_pose_ = msg->pose.pose; });

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      goal_ = *msg;
      state_ = State::PLANNING;
      computePath();
    }
  );

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
      current_map_ = *msg;
      computePath();
    }
  );
}

void PlannerNode::timerCallback() {
  if (state_ != State::PLANNING) return;
  double delta = std::hypot(goal_.point.x - robot_pose_.position.x, goal_.point.y - robot_pose_.position.y);

  if (delta < PlannerNode::completed_threshold) {
    RCLCPP_INFO(this->get_logger(), "GOAL reached!");
    state_ = State::NO_GOAL;
  } else {
    RCLCPP_INFO(this->get_logger(), "replanning...");
    computePath();
}
}

PlannerNode::CellIndex PlannerNode::worldToCell(double wx, double wy) const {
  const auto &map_info = current_map_.info;
  double gx = std::floor((wx - map_info.origin.position.x) / map_info.resolution);
  double gy = std::floor((wy - map_info.origin.position.y) / map_info.resolution);
  return CellIndex(gx, gy);
}

void PlannerNode::computePath() {
  if (state_ != State::PLANNING || current_map_.data.empty()) return;

  const CellIndex current_cell = worldToCell(robot_pose_.position.x, robot_pose_.position.y);
  const CellIndex target_cell = worldToCell(goal_.point.x, goal_.point.y);

  RCLCPP_INFO(this->get_logger(), "[A-STAR] current cell (%d, %d)", current_cell.x, current_cell.y);
  RCLCPP_INFO(this->get_logger(), "[A-STAR] target cell (%d, %d)", target_cell.x, target_cell.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  open_set.emplace(current_cell, current_cell.dist(target_cell));
  g_score[current_cell] = 0.0f;

  // A* implementation
  const int dcell = 1;
  const auto &map_info = current_map_.info;
  const double cell_threshold = PlannerNode::completed_threshold / map_info.resolution;

  std::optional<CellIndex> best;

  int nodes_searched = 0;
  while (! open_set.empty()) {
    const AStarNode cur = open_set.top();
    open_set.pop();

    if (cur.index.dist(target_cell) < cell_threshold) {
      best = cur.index;
      break;
    }

    if (closed_set.count(cur.index)) continue;

    // find neighbours
    for (int i=-dcell; i<=dcell; ++i) {
      for (int j=-dcell; j<=dcell; ++j) {
        if (i == 0 && j == 0) continue;

        int nx = cur.index.x + j, ny = cur.index.y + i;
        if (nx < 0 || ny < 0 || nx >= (int)map_info.width || ny >= (int)map_info.height) continue;
        if (current_map_.data[ny*map_info.width+nx] > 5) continue;

        const CellIndex neighbour = CellIndex(nx, ny);

        if (! g_score.count(neighbour))
          g_score[neighbour] = std::numeric_limits<double>::infinity();

        double new_g = g_score[cur.index] + cur.index.dist(neighbour);
        if (new_g > g_score[neighbour]) continue;

        came_from[neighbour] = cur.index;
        g_score[neighbour] = new_g;
        AStarNode node = AStarNode(neighbour, new_g + neighbour.dist(target_cell));

        if (! closed_set.count(neighbour))
          open_set.push(node);
      }
    }

    closed_set.insert(cur.index);
    nodes_searched++;
  }

  if (! best.has_value()) {
    RCLCPP_WARN(this->get_logger(), "[A-STAR] could NOT find a path!!");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "[A-STAR] path found!");
  }
  RCLCPP_INFO(this->get_logger(), "%d nodes considered.", nodes_searched);

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  CellIndex latest = best.value();
  auto now = this->get_clock()->now();
  while (latest != current_cell) {
    // convert to global coordinate frame
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = now;
    pose.header.frame_id = "sim_world";
    pose.pose.position.x = ((double)latest.x * map_info.resolution) + map_info.origin.position.x;
    pose.pose.position.y = ((double)latest.y * map_info.resolution) + map_info.origin.position.y;

    poses.push_back(pose);
    latest = came_from[latest];
  }
  std::reverse(poses.begin(), poses.end());

  auto path = nav_msgs::msg::Path();
  path.header.stamp = now;
  path.header.frame_id = "sim_world";
  path.poses = std::move(poses);

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
