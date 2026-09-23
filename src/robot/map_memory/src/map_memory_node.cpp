#include <chrono>
#include "map_memory_node.hpp"
#include <geometry_msgs/msg/point.hpp>

MapMemoryNode::MapMemoryNode() : Node("map_memory") {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::processCostmap, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::processOdometry, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::transmitMap, this));

  global_map_ = nav_msgs::msg::OccupancyGrid();
  global_map_.info.resolution = MapMemoryNode::map_resolution;
  global_map_.info.width = MapMemoryNode::map_size / MapMemoryNode::map_resolution;
  global_map_.info.height = MapMemoryNode::map_size / MapMemoryNode::map_resolution;
  global_map_.info.origin.position.x = -(double)MapMemoryNode::map_size / 2.0;
  global_map_.info.origin.position.y = -(double)MapMemoryNode::map_size / 2.0;
  global_map_.header.frame_id = "sim_world";
  global_map_.data.assign(std::pow(global_map_.info.width, 2), 0);
}

void MapMemoryNode::processCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;

  latest_x = p.x;
  latest_y = p.y;
  latest_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double dist = std::sqrt(std::pow(latest_x - last_x, 2) + std::pow(latest_y - last_y, 2));
  if (dist > MapMemoryNode::distance_threshold) {
    last_x = latest_x;
    last_y = latest_y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::transmitMap() {
  if (! (should_update_map_ && costmap_updated_)) return;

  // integrate costmap
  const auto &gi = global_map_.info;
  const auto &ci = latest_costmap_.info;
  const double ctheta = std::cos(latest_yaw), stheta = std::sin(latest_yaw);

  for (int cy=0; cy<(int)ci.height; ++cy) {
    for (int cx=0; cx<(int)ci.width; ++cx) {
      int8_t cost = latest_costmap_.data[cy*ci.width + cx];
      if (cost <= 0) continue;

      double robot_x = ci.origin.position.x + cx * ci.resolution;
      double robot_y = ci.origin.position.y + cy * ci.resolution;

      // robot -> world frame, 2d coordinate frame transformation
      double world_x = latest_x + (robot_x*ctheta - robot_y*stheta);
      double world_y = latest_y + (robot_x*stheta + robot_y*ctheta);

      int map_x = std::floor((world_x - gi.origin.position.x) / gi.resolution);
      int map_y = std::floor((world_y - gi.origin.position.y) / gi.resolution);
      if (map_x < 0 || map_y < 0 || map_x >= (int)gi.width || map_y >= (int)gi.height) continue;

      global_map_.data[map_y*gi.width+map_x] = std::max(cost, global_map_.data[map_y*gi.width+map_x]);
    }
  }

  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);

  should_update_map_ = false;
  costmap_updated_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
