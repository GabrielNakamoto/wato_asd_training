#include <memory>
#include "costmap_node.hpp"
#include <nav_msgs/msg/map_meta_data.hpp>
#include <std_msgs/msg/header.hpp>

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::processLidar, this, std::placeholders::_1));
}

void CostmapNode::processLidar(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan) {
  const int n_cells = CostmapNode::grid_size * CostmapNode::grid_resolution;
  std::vector<int8_t> occupancy_grid(n_cells*n_cells, 0);
  std::vector<std::size_t> detected;

  // initial distance detection
  for (std::size_t i=0; i<scan->ranges.size(); ++i) {
    double theta = scan->angle_min + i * scan->angle_increment, range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      double x_proj = range * std::cos(theta), y_proj = range * std::sin(theta);
      int x_idx = x_proj * (int)grid_resolution + (n_cells/2);
      int y_idx = y_proj * (int)grid_resolution + (n_cells/2);

      if (x_idx < 0 || y_idx < 0 || y_idx >= n_cells || x_idx >= n_cells) continue;

      occupancy_grid[y_idx * n_cells + x_idx] = 100;
      detected.push_back(y_idx * n_cells + x_idx);
    }
  }

  // propogate propability based on proximity
  const int inflate_dist = CostmapNode::inflation_radius * CostmapNode::grid_resolution;
  for (const std::size_t &cell : detected) {
    const int x = static_cast<int>(cell) % n_cells, y = static_cast<int>(cell) / n_cells;

    for (int dy=-inflate_dist; dy<=inflate_dist; ++dy) {
      for (int dx=-inflate_dist; dx<=inflate_dist; ++dx) {
        int nx = x+dx, ny = y+dy;
        if (nx < 0 || ny < 0 || nx >= n_cells || ny >= n_cells) continue;

        // NOTE: rate is done in cell space not meters
        const std::size_t eudist = std::sqrt(dy*dy + dx*dx);
        if (eudist > inflate_dist) continue;
        uint8_t cost = static_cast<uint8_t>(CostmapNode::inflation_max_cost * (1.0 - ((double)eudist/inflate_dist)));

        if (cost > occupancy_grid[ny*n_cells + nx])
          occupancy_grid[ny*n_cells + nx] = cost;
      }
    }
  }

  // serialize
  auto grid_packet = nav_msgs::msg::OccupancyGrid();
  grid_packet.info.width = n_cells;
  grid_packet.info.height = n_cells;
  grid_packet.info.resolution = 1.0f / (float)CostmapNode::grid_resolution;
  grid_packet.info.origin.position.x = -(double)grid_size / 2.0;
  grid_packet.info.origin.position.y = -(double)grid_size / 2.0;

  grid_packet.header.stamp = this->get_clock()->now();
  grid_packet.header.frame_id = scan->header.frame_id;

  grid_packet.data = std::move(occupancy_grid);

  // transmit
  grid_pub_->publish(grid_packet);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
